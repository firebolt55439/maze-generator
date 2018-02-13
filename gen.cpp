#include <iostream>
#include <cassert>
#include <string>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stack>
#include <algorithm>
#include <list>
#include <random>
#include <vector>
#include <set>
#include <tuple>
#include <map>
#include <queue>

extern "C" {
    #include <libavutil/avassert.h>
    #include <libavutil/channel_layout.h>
    #include <libavutil/opt.h>
    #include <libavutil/mathematics.h>
    #include <libavutil/timestamp.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libswresample/swresample.h>
}

int MAZE_WIDTH = 160;
int MAZE_HEIGHT = 90;
int MAZE_SIDE_MULTIPLIER = 4;
int MAZE_FRAMES_PER_ITERATION = 3;
int MAZE_SOLVE_FRAMES_PER_ITERATION = 2;
int MAZE_ITERATIONS_PER_SECOND = 20;
int STREAM_FRAME_RATE = (MAZE_FRAMES_PER_ITERATION * MAZE_ITERATIONS_PER_SECOND); // in frames/second

#define STREAM_PIX_FMT    AV_PIX_FMT_YUV420P /* default pix_fmt */
#define SCALE_FLAGS SWS_BICUBIC

// a wrapper around a single output AVStream
typedef struct OutputStream {
    AVStream *st;
    AVCodecContext *enc;
    /* pts of the next frame that will be generated */
    int64_t next_pts;
    int samples_count;
    AVFrame *frame;
    AVFrame *tmp_frame;
    float t, tincr, tincr2;
    struct SwsContext *sws_ctx;
    struct SwrContext *swr_ctx;
} OutputStream;

static void log_packet(const AVFormatContext *fmt_ctx, const AVPacket *pkt)
{
    return;
    AVRational *time_base = &fmt_ctx->streams[pkt->stream_index]->time_base;
    printf("pts:%s pts_time:%s dts:%s dts_time:%s duration:%s duration_time:%s stream_index:%d\n",
           av_ts2str(pkt->pts), av_ts2timestr(pkt->pts, time_base),
           av_ts2str(pkt->dts), av_ts2timestr(pkt->dts, time_base),
           av_ts2str(pkt->duration), av_ts2timestr(pkt->duration, time_base),
           pkt->stream_index);
}
static int write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt)
{
    /* rescale output packet timestamp values from codec to stream timebase */
    av_packet_rescale_ts(pkt, *time_base, st->time_base);
    pkt->stream_index = st->index;
    /* Write the compressed frame to the media file. */
    log_packet(fmt_ctx, pkt);
    return av_interleaved_write_frame(fmt_ctx, pkt);
}
/* Add an output stream. */
uint8_t* rgb24Data;
static void add_stream(OutputStream *ost, AVFormatContext *oc,
                       AVCodec **codec,
                       enum AVCodecID codec_id)
{
    AVCodecContext *c;
    int i;
    /* find the encoder */
    *codec = avcodec_find_encoder(codec_id);
    if (!(*codec)) {
        fprintf(stderr, "Could not find encoder for '%s'\n",
                avcodec_get_name(codec_id));
        exit(1);
    }
    ost->st = avformat_new_stream(oc, NULL);
    if (!ost->st) {
        fprintf(stderr, "Could not allocate stream\n");
        exit(1);
    }
    ost->st->id = oc->nb_streams-1;
    c = avcodec_alloc_context3(*codec);
    if (!c) {
        fprintf(stderr, "Could not alloc an encoding context\n");
        exit(1);
    }
    ost->enc = c;
    switch ((*codec)->type) {
    case AVMEDIA_TYPE_VIDEO:
        c->codec_id = codec_id;
        c->bit_rate = 400000;
        /* Resolution must be a multiple of two. */
        c->width    = MAZE_WIDTH * MAZE_SIDE_MULTIPLIER;
        c->height   = MAZE_HEIGHT * MAZE_SIDE_MULTIPLIER;
        rgb24Data = new uint8_t[3*c->width*c->height];
        /* timebase: This is the fundamental unit of time (in seconds) in terms
         * of which frame timestamps are represented. For fixed-fps content,
         * timebase should be 1/framerate and timestamp increments should be
         * identical to 1. */
        ost->st->time_base = (AVRational){ 1, STREAM_FRAME_RATE };
        c->time_base       = ost->st->time_base;
        c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
        c->pix_fmt       = STREAM_PIX_FMT;
        if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
            /* just for testing, we also add B-frames */
            c->max_b_frames = 2;
        }
        if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
            /* Needed to avoid using macroblocks in which some coeffs overflow.
             * This does not happen with normal video, it just happens here as
             * the motion of the chroma plane does not match the luma plane. */
            c->mb_decision = 2;
        }
    break;
    default:
        break;
    }
    /* Some formats want stream headers to be separate. */
    if (oc->oformat->flags & AVFMT_GLOBALHEADER)
        c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
}
/**************************************************************/
/* video output */
static AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    int ret;
    picture = av_frame_alloc();
    if (!picture)
        return NULL;
    picture->format = pix_fmt;
    picture->width  = width;
    picture->height = height;
    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 32);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate frame data.\n");
        exit(1);
    }
    return picture;
}
static void open_video(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg)
{
    int ret;
    AVCodecContext *c = ost->enc;
    AVDictionary *opt = NULL;
    av_dict_copy(&opt, opt_arg, 0);
    /* open the codec */
    ret = avcodec_open2(c, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        fprintf(stderr, "Could not open video codec: %s\n", av_err2str(ret));
        exit(1);
    }
    /* allocate and init a re-usable frame */
    ost->frame = alloc_picture(c->pix_fmt, c->width, c->height);
    if (!ost->frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }
    /* If the output format is not YUV420P, then a temporary YUV420P
     * picture is needed too. It is then converted to the required
     * output format. */
    ost->tmp_frame = NULL;
    if (c->pix_fmt != AV_PIX_FMT_YUV420P) {
        ost->tmp_frame = alloc_picture(AV_PIX_FMT_YUV420P, c->width, c->height);
        if (!ost->tmp_frame) {
            fprintf(stderr, "Could not allocate temporary picture\n");
            exit(1);
        }
    }
    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        fprintf(stderr, "Could not copy the stream parameters\n");
        exit(1);
    }
}
typedef enum {
    UP = 3,
    DOWN = 2,
    LEFT = 1,
    RIGHT = 0
} Direction;
std::stack<std::tuple<int, Direction> > stk;
int curPos = -1;
bool isMazeDone = false;
std::set<int> mazeData;
inline int ind_from_pos(int x, int y){
    return y * MAZE_WIDTH + x;
}
inline bool is_valid(int x, int y){
    return (x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT);
}
int lookInDirection(int ind, Direction d, int n = 1){
    static int dx[] = {1, -1, 0, 0};
    static int dy[] = {0, 0, 1, -1};
    int y = ind / MAZE_WIDTH, x = ind % MAZE_WIDTH;
    y += dy[d] * n; x += dx[d] * n;
    if(is_valid(x, y)) return ind_from_pos(x, y);
    else return -1;
}
std::list<std::tuple<int, Direction> > getNeighbors(int ind){
    int r = ind / MAZE_WIDTH, c = ind % MAZE_WIDTH;
    std::list<std::tuple<int, Direction> > ret;
    for(int i = 0; i < 4; i++){
        Direction d = Direction(i);
        int one = lookInDirection(ind, d, 1);
        int two = lookInDirection(ind, d, 2);
        if(one != -1 && two != -1 && !mazeData.count(one) && !mazeData.count(two)){
            ret.push_back(std::make_tuple(one, d));
        }
    }
    return ret;
}
bool should_solve = false;
bool should_solve_after = false;
bool showing_path = false;
int solve_destination = -1;
std::set<int> visited;
std::map<int, int> parent;
std::set<int> current_path;
std::stack<int> entire_path;
void maze_solve_iteration(int ind){
    // ind = y * width + x
    if(ind % MAZE_SOLVE_FRAMES_PER_ITERATION) return;
    if(showing_path){
        if(entire_path.size() == 0){
            isMazeDone = true;
        } else {
            curPos = entire_path.top();
            entire_path.pop();
            current_path.insert(curPos);
            printf("winding down stack size %lu\n", entire_path.size());
        }
        return;
    }
    if(stk.empty() && ind > 0){
        isMazeDone = true;
        return;
    } else if(stk.empty()){
        stk.push(std::make_tuple(ind_from_pos(0, 0), DOWN));
    }
    curPos = std::get<0>(stk.top());
    stk.pop();
    if(visited.count(curPos)){
        maze_solve_iteration(ind);
        return;
    }
    printf(
        "r: %d | c : %d | stk size: %lu\n",
        curPos / MAZE_WIDTH, curPos % MAZE_WIDTH, stk.size()
    );
    visited.insert(curPos);
    if(curPos == solve_destination){
        showing_path = true;
        do {
            entire_path.push(curPos);
            curPos = parent[curPos];
        } while(curPos != 0);
        entire_path.push(curPos);
        curPos = solve_destination;
        return;
    }
    std::vector<std::tuple<int, Direction> > potential;
    for(int i = 0; i < 4; i++){
        Direction d = Direction(i);
        int lookAhead = lookInDirection(curPos, d);
        if(lookAhead != -1 && mazeData.count(lookAhead) && !visited.count(lookAhead)){
            parent[lookAhead] = curPos;
            potential.push_back(std::make_tuple(lookAhead, d));
        }
    }
    std::sort(potential.begin(), potential.end(), [](auto a, auto b){
        int av = std::get<0>(a), bv = std::get<0>(b);
        int ra = av / MAZE_WIDTH, rb = bv / MAZE_WIDTH;
        int ca = av % MAZE_WIDTH, cb = bv % MAZE_WIDTH;
        if(ca < cb) return true; // reversed due to ascending order + stack pop mechanism
        if(ca > cb) return false;
        if(ra < rb) return true;
        return false;
    });
    for(auto on : potential){
        stk.push(on);
    }
}
void maze_iteration(int ind){
    // ind = y * width + x
    if(ind % MAZE_FRAMES_PER_ITERATION) return;
    if(stk.empty() && ind > 0){
        isMazeDone = true;
        return;
    } else if(stk.empty()){
        stk.push(std::make_tuple(ind_from_pos(0, 0), DOWN));
    }
    auto curTup = stk.top();
    curPos = std::get<0>(curTup);
    stk.pop();
    int next = curPos;
    if(curPos){
        next = lookInDirection(curPos, std::get<1>(curTup), 1);
        //printf("Added %d in direction %d\n", next, std::get<1>(curTup));
        assert(next != -1);
        if(mazeData.count(next)){
            maze_iteration(ind);
            return;
        }
        mazeData.insert(next);
    }
    mazeData.insert(curPos);
    curPos = next;
    auto pos = getNeighbors(curPos);
    if(!pos.size()){
        maze_iteration(ind);
        return;
    }
    printf(
        "r: %d | c : %d | d: %d | n: %lu | stk size: %lu\n",
        curPos / MAZE_WIDTH, curPos % MAZE_WIDTH, std::get<1>(curTup), pos.size(), stk.size()
    );
    std::vector<std::tuple<int, Direction> > filtered(pos.begin(), pos.end());
    std::shuffle(filtered.begin(), filtered.end(), std::default_random_engine(rand() * ind));
    for(auto on : filtered){
        stk.push(on);
    }
}
bool generate_only = false;
int gen_counter = 0;
static void fill_yuv_image(AVFrame *pict, int frame_index, int width, int height){
    int x, y, i, j, r, g, b;
    if(!generate_only){
        if(!should_solve) maze_iteration(frame_index);
        else maze_solve_iteration(frame_index);
    }
    SwsContext* ctx = sws_getContext(
        width, height,
        AV_PIX_FMT_RGB24, width, height,
        AV_PIX_FMT_YUV420P, 0, 0, 0, 0
    );
    int mult = MAZE_SIDE_MULTIPLIER;
    for(y = 0; y < height * 3; y += 3){
        j = y / 3;
        for(x = 0; x < width; x++){
            i = x;
            r = 0, g = 0, b = 0;
            //printf("(%d, %d) | (%d, %d)\n", i, j, i / mult, j / mult);
            int on_now = ind_from_pos(i / mult, j / mult);
            if(mazeData.count(on_now)){
                //printf("HIT at (%d, %d) --> %d\n", i / mult, j / mult, ind_from_pos(i / mult, j / mult));
                r = g = b = 255;
            }
            if(curPos == on_now && !isMazeDone){
                r = 255;
                g = b = 0;
            }
            if(should_solve){
                if(visited.count(on_now)){
                    r = b = 0;
                    g = 255;
                }
                if(curPos == on_now && !isMazeDone){
                    b = 255;
                    r = g = 0;
                }
            }
            if(showing_path){
                if(current_path.count(on_now)){
                    r = 255;
                    g = b = 0;
                }
            }
            rgb24Data[y * width + x * 3 + 0] = r; // R
            rgb24Data[y * width + x * 3 + 1] = g; // G
            rgb24Data[y * width + x * 3 + 2] = b; // B
            if((y * width + x * 3 + 2) >= (3 * width * height)){
                fprintf(stderr, "Memory busted.\n");
                ::exit(1);
            }
        }
    }
    //::exit(1);
    uint8_t* inData[1] = { rgb24Data };
    int inLineSize[1] = { 3*width };
    sws_scale(ctx, inData, inLineSize, 0, height, pict->data, pict->linesize);
}
bool last_frame_done = false;
static AVFrame *get_video_frame(OutputStream *ost){
    AVCodecContext *c = ost->enc;
    /* check if we want to generate more frames */
    if(isMazeDone){
        if(generate_only){
            ++gen_counter;
            if(gen_counter > STREAM_FRAME_RATE) return NULL;
        } else if((should_solve && current_path.size() == 0) || should_solve_after){
            int last_sq = ind_from_pos(MAZE_WIDTH - 1, MAZE_HEIGHT - 1);
            if(!mazeData.count(last_sq)){
                printf("moved left1\n");
                last_sq = lookInDirection(last_sq, LEFT);
            }
            if(!mazeData.count(last_sq)){
                printf("moved up + right\n");
                last_sq = lookInDirection(last_sq, UP);
                last_sq = lookInDirection(last_sq, RIGHT);
            }
            if(!mazeData.count(last_sq)){
                printf("moved left2\n");
                last_sq = lookInDirection(last_sq, LEFT);
            }
            solve_destination = last_sq;
            printf("Solve destination: (%d, %d)\n", solve_destination % MAZE_WIDTH, solve_destination / MAZE_WIDTH);
            isMazeDone = false;
            if(!should_solve && should_solve_after){
                should_solve_after = false;
                should_solve = true;
                stk.push(std::make_tuple(ind_from_pos(0, 0), DOWN));
            }
        } else {
            if(last_frame_done){
                return NULL;
            }
            last_frame_done = true;
        }
    }
    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0)
        exit(1);
    fill_yuv_image(ost->frame, ost->next_pts, c->width, c->height);
    ost->frame->pts = ost->next_pts++;
    return ost->frame;
}
/*
 * encode one video frame and send it to the muxer
 * return 1 when encoding is finished, 0 otherwise
 */
static int write_video_frame(AVFormatContext *oc, OutputStream *ost)
{
    int ret;
    AVCodecContext *c;
    AVFrame *frame;
    int got_packet = 0;
    AVPacket pkt = { 0 };
    c = ost->enc;
    frame = get_video_frame(ost);
    av_init_packet(&pkt);
    /* encode the image */
    ret = avcodec_encode_video2(c, &pkt, frame, &got_packet);
    if (ret < 0) {
        fprintf(stderr, "Error encoding video frame: %s\n", av_err2str(ret));
        exit(1);
    }
    if (got_packet) {
        ret = write_frame(oc, &c->time_base, ost->st, &pkt);
    } else {
        ret = 0;
    }
    if (ret < 0) {
        fprintf(stderr, "Error while writing video frame: %s\n", av_err2str(ret));
        exit(1);
    }
    return (frame || got_packet) ? 0 : 1;
}
static void close_stream(AVFormatContext *oc, OutputStream *ost)
{
    avcodec_free_context(&ost->enc);
    av_frame_free(&ost->frame);
    av_frame_free(&ost->tmp_frame);
    sws_freeContext(ost->sws_ctx);
    swr_free(&ost->swr_ctx);
}
/**************************************************************/
/* media file output */
int main(int argc, char **argv)
{
    srand(time(NULL));
    av_log_set_level(AV_LOG_WARNING);
    av_register_all();
    avcodec_register_all();
    OutputStream video_st = { 0 }, audio_st = { 0 };
    const char *filename;
    AVOutputFormat *fmt;
    AVFormatContext *oc = NULL;
    AVCodec *audio_codec, *video_codec;
    int ret;
    int have_video = 0, have_audio = 0;
    int encode_video = 0, encode_audio = 0;
    AVDictionary *opt = NULL;
    int i;
    if (argc < 2) {
        printf("usage: %s output_file\n"
               "-wWIDTH set maze width\n"
               "-hHEIGHT set maze height\n"
               "--generate-only show only the generated maze\n"
               "--solve-only show only the maze solving, not the maze generation\n"
               "--solve-after solve the maze after generating it\n"
               "-smSIDE_MULTIPLIER square root of pixels per maze square (higher = slower but better resolution)\n"
               "-fpiFRAMES_PER_ITERATION how many frames per iteration of the maze generation algorithm\n"
               "-sfpiSOLVE_FRAMES_PER_ITERATION how many frames per iteration of the maze solving algorithm\n"
               "-ipsITERATIONS_PER_SECOND adjust frame rate to show the specified iterations per second\n"
               "\n", argv[0]);
        return 1;
    }
    filename = argv[1];
    for (i = 2; i+1 < argc; i+=2) {
        if (!strcmp(argv[i], "-flags") || !strcmp(argv[i], "-fflags"))
            av_dict_set(&opt, argv[i]+1, argv[i+1], 0);
    }
    for(int i = 2; i < argc; i++){
        std::string on(argv[i]);
        if(!strcmp(argv[i], "--solve-only"))
            should_solve = true;
        else if(!strcmp(argv[i], "--solve-after"))
            should_solve_after = true;
        else if(on.find("-w") == 0)
            MAZE_WIDTH = atoi(on.substr(2).c_str());
        else if(on.find("-h") == 0)
            MAZE_HEIGHT = atoi(on.substr(2).c_str());
        else if(on == "--generate-only")
            generate_only = true;
        else if(on.find("-sm") == 0)
            MAZE_SIDE_MULTIPLIER = atoi(on.substr(3).c_str());
        else if(on.find("-fpi") == 0)
            MAZE_FRAMES_PER_ITERATION = atoi(on.substr(4).c_str());
        else if(on.find("-ips") == 0)
            MAZE_ITERATIONS_PER_SECOND = atoi(on.substr(4).c_str());
        else if(on.find("-sfpi") == 0)
            MAZE_SOLVE_FRAMES_PER_ITERATION = atoi(on.substr(5).c_str());
    }
    STREAM_FRAME_RATE = (MAZE_FRAMES_PER_ITERATION * MAZE_ITERATIONS_PER_SECOND);
    if(!STREAM_FRAME_RATE || !MAZE_FRAMES_PER_ITERATION || !MAZE_ITERATIONS_PER_SECOND || !MAZE_SOLVE_FRAMES_PER_ITERATION){
        fprintf(stderr, "Rates cannot be 0.\n");
        return 1;
    }
    if(generate_only){
        int ind = 0;
        while(!isMazeDone){
            maze_iteration(ind++);
        }
    }
    if(should_solve && should_solve_after){
        fprintf(stderr, "Cannot only solve and solve after generation simultaneously.\n");
        return 1;
    }
    if(should_solve){
        int i = 0;
        while(!isMazeDone){
            maze_iteration(i++);
        }
    }
    /* allocate the output media context */
    int err_code = avformat_alloc_output_context2(&oc, NULL, NULL, filename);
    if (!oc) {
        fprintf(stderr, "err: %d |%s|\n", err_code, av_err2str(err_code));
        printf("Could not deduce output format from file extension: using MPEG.\n");
        avformat_alloc_output_context2(&oc, NULL, "mpeg", filename);
    }
    if (!oc)
        return 1;
    fmt = oc->oformat;
    /* Add the audio and video streams using the default format codecs
     * and initialize the codecs. */
    if (fmt->video_codec != AV_CODEC_ID_NONE) {
        add_stream(&video_st, oc, &video_codec, fmt->video_codec);
        have_video = 1;
        encode_video = 1;
    }
    /* Now that all the parameters are set, we can open the audio and
     * video codecs and allocate the necessary encode buffers. */
    if (have_video)
        open_video(oc, video_codec, &video_st, opt);
    av_dump_format(oc, 0, filename, 1);
    /* open the output file, if needed */
    if (!(fmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&oc->pb, filename, AVIO_FLAG_WRITE);
        if (ret < 0) {
            fprintf(stderr, "Could not open '%s': %s\n", filename,
                    av_err2str(ret));
            return 1;
        }
    }
    /* Write the stream header, if any. */
    ret = avformat_write_header(oc, &opt);
    if (ret < 0) {
        fprintf(stderr, "Error occurred when opening output file: %s\n",
                av_err2str(ret));
        return 1;
    }
    while (encode_video || encode_audio) {
        /* select the stream to encode */
        if (encode_video &&
            (!encode_audio || av_compare_ts(video_st.next_pts, video_st.enc->time_base,
                                            audio_st.next_pts, audio_st.enc->time_base) <= 0)) {
            encode_video = !write_video_frame(oc, &video_st);
        }
    }
    /* Write the trailer, if any. The trailer must be written before you
     * close the CodecContexts open when you wrote the header; otherwise
     * av_write_trailer() may try to use memory that was freed on
     * av_codec_close(). */
    av_write_trailer(oc);
    /* Close each codec. */
    if (have_video)
        close_stream(oc, &video_st);
    if (have_audio)
        close_stream(oc, &audio_st);
    if (!(fmt->flags & AVFMT_NOFILE))
        /* Close the output file. */
        avio_closep(&oc->pb);
    /* free the stream */
    avformat_free_context(oc);
    delete[] rgb24Data;
    return 0;
}