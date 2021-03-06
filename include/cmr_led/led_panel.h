#ifndef LED_PANEL_H
#define LED_PANEL_H



#include "ros/ros.h"
#include "cmr_led/img_loader.h" 
#include "led-matrix.h"
#include "pixel-mapper.h"
#include "content-streamer.h"

#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <queue> 
#include <ros/package.h>

#include "std_srvs/Empty.h"

#include <cmr_msgs/SetLEDExpression.h>
#include <cmr_msgs/EyeFollowerPersons.h>
#include <cmr_os/cmr_holder.h>
#include <functional>


using rgb_matrix::GPIO;
using rgb_matrix::Canvas;
using rgb_matrix::FrameCanvas;
using rgb_matrix::RGBMatrix;
using rgb_matrix::StreamReader;
using rgb_matrix::GPIO;
using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

using namespace std;

static const uint32_t LED_COLS = 64;
static const uint32_t LED_ROWS = 32;
static constexpr double MAX_EYE_ANGLE = 70.;
static constexpr double MAX_EYE_DIST = 2.5;
static constexpr double MAX_DUR_PERSON = 3.0;

using LEDEx = cmr_msgs::LEDExpression;

using IMGPair = std::pair<shared_ptr<BMP>,shared_ptr<GIF>>;
using DirectionMap = std::map<int, IMGPair>;
using ExpressionMap = std::map<int,DirectionMap>;

using TransitionMap = std::map<int,std::map<int,shared_ptr<GIF>>>;
using ExprGifMap = std::map<int,shared_ptr<GIF>>;
using ExprStringMap = std::map<int,std::string>;

class LEDPanel
{
public:
    LEDPanel(ros::NodeHandle &node_handle);
   ~LEDPanel();
    void cleanup();
private:
   typedef enum {tBMP, tGIF} imgtype;

    struct GIFPlaybackOpts{
	GIFPlaybackOpts(){
		gif = std::make_shared<GIF>(LED_COLS, LED_ROWS,1);
	}
	void reset(){

		cycles = 1;
		cycles_completed = 0;
		current_frame = 0;
		is_playing = false;
	}
	shared_ptr<GIF> gif;
	uint32_t cycles = 1;
	uint32_t cycles_completed = 0;
	uint32_t current_frame = 0;
	bool is_playing = false;
    }gif_opts_;

    struct EyeAngleOpts{
    	int num = 0;
	ros::Time time_set;
    }eye_angle_opts_;

    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Publisher my_publisher_;
    ros::Subscriber sub_persons_;
    ros::Timer tim_gif_play_;
    ros::Timer tim_blink_;
    ros::Timer tim_main_;
    ros::ServiceServer serv_set_expression_;

    ros::Time time_expression_reset_;

   //RGB Matrix
    RGBMatrix::Options rgbm_defaults_;
    rgb_matrix::RuntimeOptions rgbm_runtime_defaults_;
    Canvas *canvas_;

    int current_expression_ = LEDEx::NEUTRAL;

    IMGLoader *img_loader_;

    std::string path_img_base_;
    ExpressionMap expr_map_;
    TransitionMap trans_map_;
    ExprStringMap expr_str_map_;
    std::queue<std::function<void()>> gif_play_queue_;

    cmr_os::cmrHolder<shared_ptr<BMP>> bmp_holder_;

    void writeToMatrix(const RGBArray2D &rgb_array);
    void playGif(shared_ptr<GIF> gif, uint32_t cycles = 1);
    void queueExpression(int expression, double duration);
    void loadExpressions();
    void loadTransitions();
    int getNumFromAngle(const double &angle);
    bool angleInRange(const double &angle);
    void stopGifAndShowBMP(shared_ptr<BMP> bmp);
    bool expressionIsOver(){return (ros::Time::now() - time_expression_reset_).toSec() > 0;}

    std::string getFileNameFromExpression(int expression, int direction, imgtype type);
    std::string getTransitionFileNameFromExpressions(int expression_1, int expression_2, int direction);

    DirectionMap initDirMap();
    ExprGifMap initExprGifMap();
    unsigned int num_detections_ = 0;
    

    // callbacks
    void subPersonsCallback(const cmr_msgs::EyeFollowerPersonsConstPtr &msg);
    void timerGifPlayCallback(const ros::TimerEvent &evt);
    void timerBlinkCallback(const ros::TimerEvent &evt);
    void timerMainCallback(const ros::TimerEvent &evt);
    bool servSetLEDExpressionCallback(cmr_msgs::SetLEDExpression::Request &req, cmr_msgs::SetLEDExpression::Response &res);

    //variables

};


#endif // LED_PANEL_H
