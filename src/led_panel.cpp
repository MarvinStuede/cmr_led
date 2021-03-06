#include "cmr_led/led_panel.h"

LEDPanel::LEDPanel(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
  //Write the parameters of the LED Panel
  rgbm_defaults_.rows = LED_ROWS;
  rgbm_defaults_.cols = LED_COLS;
  rgbm_defaults_.chain_length = 1;
  rgbm_defaults_.parallel = 1;
  rgbm_defaults_.hardware_mapping = "regular";
  rgbm_defaults_.led_rgb_sequence = "RGB";
  rgbm_defaults_.disable_hardware_pulsing = true;

  //Fill the map with string names of the expressions (used for file naming)
  expr_str_map_[LEDEx::NEUTRAL] = "neutral";
  expr_str_map_[LEDEx::SAD] = "sad";
  expr_str_map_[LEDEx::SLEEP] = "sleep";
  expr_str_map_[LEDEx::ERROR] = "error";
  expr_str_map_[LEDEx::ANGRY] = "angry";
  expr_str_map_[LEDEx::LAUGHING] = "laughing";
  expr_str_map_[LEDEx::CONFUSED] = "confused";
  expr_str_map_[LEDEx::CURIOUS] = "curious"; //NEW
  expr_str_map_[LEDEx::PROCESS] = "process"; //NEW

  //Create the canvas object for access of the panel
  canvas_ = rgb_matrix::CreateMatrixFromOptions(rgbm_defaults_,rgbm_runtime_defaults_);
  if (canvas_ == NULL) {
    PrintMatrixFlags(stderr, rgbm_defaults_, rgbm_runtime_defaults_);
  }
  //Set the path of the BMPs and GIFs
  //TODO: Launch param
  path_img_base_ = "/home/pi/catkin_ws_led/src/cmr_led/cfg/img/";
  img_loader_ = new IMGLoader();

  //Load the images and animations
  loadExpressions();
  loadTransitions();

  //Set the neutral image facing front as image to show
  bmp_holder_.set_value(expr_map_[LEDEx::NEUTRAL][0].first);
  eye_angle_opts_.time_set = ros::Time::now();

  //Create ROS objects
  tim_gif_play_ = node_->createTimer(ros::Duration(0.05), &LEDPanel::timerGifPlayCallback, this);
  tim_blink_ = node_->createTimer(ros::Duration(5.0), &LEDPanel::timerBlinkCallback, this);
  tim_main_ = node_->createTimer(ros::Duration(0.05), &LEDPanel::timerMainCallback, this);
  serv_set_expression_ = node_->advertiseService("/sobi/led_panel/expression", &LEDPanel::servSetLEDExpressionCallback, this);
  sub_persons_ = node_->subscribe("/sobi/eye_follower_persons", 10, &LEDPanel::subPersonsCallback, this);


  tim_gif_play_.stop();

}

LEDPanel::~LEDPanel(){
  cleanup();
}
void LEDPanel::cleanup(){
  ROS_WARN("Deleting LED");
  stopGifAndShowBMP(expr_map_[LEDEx::NEUTRAL][0].first);
  canvas_->Clear();
  delete canvas_;
  delete img_loader_;

}

void LEDPanel::writeToMatrix(const RGBArray2D &rgb_array){

  if(&rgb_array == NULL){
    ROS_ERROR("RGB Array to write is NULL");
    return;
  }

  //Always clear the canvas before writing a new image
  canvas_->Clear();

  for(size_t y = 0; y < LED_ROWS; y++)
  {
    for(size_t x = 0; x < LED_COLS; x++)
    {
      canvas_->SetPixel(x, y,rgb_array(x,y).r,rgb_array(x,y).g,rgb_array(x,y).b);
    }
  }

}
void LEDPanel::playGif(shared_ptr<GIF> gif, uint32_t cycles){

  //Setup the GIF timer
  tim_gif_play_.stop();

  //Take the period of the timer from the gif
  tim_gif_play_.setPeriod(ros::Duration(gif->delay));

  gif_opts_.reset();
  gif_opts_.is_playing = true;
  gif_opts_.gif = gif;
  gif_opts_.cycles = cycles;
  tim_gif_play_.start();


}
void LEDPanel::queueExpression(int expression, double duration){

  auto gif = expr_map_[expression][0].second;
  auto trans_gif = trans_map_[current_expression_][expression];

  //Check if the transition animation exists
  if(trans_gif != NULL){
    //Calculate the cycles of the animation by substracting the transition time
    uint32_t cycles_gif = (int)(duration - 2 * trans_gif->getLength())/gif->getLength();
    gif_play_queue_.push( [=] { playGif(trans_gif,1); } );
    gif_play_queue_.push( [=] { playGif(gif,cycles_gif); } );


    trans_gif = trans_map_[expression][current_expression_];
    //Check if the transition back exists
    if(trans_gif != NULL){
      gif_play_queue_.push( [=] { playGif(trans_gif,1); } );
    }
  }
  else{
    //Just queue the animation
    gif_play_queue_.push( [=] { playGif(gif,(int)duration/gif->getLength()); } );
  }

  current_expression_ = expression;

}
void LEDPanel::loadExpressions(){
  //Initialize the map
  for_each(expr_str_map_.begin(), expr_str_map_.end(), [=](const pair<int,string> &p){expr_map_[p.first] = initDirMap();});

  for (auto it = expr_map_.begin(); it != expr_map_.end(); it++ )
  {
    for (auto it_dir = it->second.begin(); it_dir != it->second.end(); it_dir++ )
    {
      //Get absolute path of BMP and GIF
      std::string bmp_name = path_img_base_ + getFileNameFromExpression(it->first, it_dir->first, tBMP);
      std::string gif_name = path_img_base_ + getFileNameFromExpression(it->first, it_dir->first, tGIF);

      //Load the BMPs and GIFs into memory, if they exist
      //If the files do not exist, the Expression Map contains NULL at this position
      if(img_loader_->fileExists(bmp_name)){
        it_dir->second.first = img_loader_->loadBMP(bmp_name);
      }
      else it_dir->second.first.reset();

      if(img_loader_->fileExists(gif_name)){
        it_dir->second.second = img_loader_->loadGIF(gif_name);
      }
      else it_dir->second.second.reset();
    }
  }
}
void LEDPanel::loadTransitions(){
  //Initialize the map
  for_each(expr_str_map_.begin(), expr_str_map_.end(), [=](const pair<int,string> &p){trans_map_[p.first] = initExprGifMap();});

  for (auto it = trans_map_.begin(); it != trans_map_.end(); it++ )
  {
    for (auto it_eg = it->second.begin(); it_eg != it->second.end(); it_eg++ )
    {
      //Get the filename of the transition animation, only use center (4) for now
      std::string gif_name = path_img_base_ + getTransitionFileNameFromExpressions(it->first,it_eg->first,4);

      if(img_loader_->fileExists(gif_name))
      {
        it_eg->second = img_loader_->loadGIF(gif_name);
      }
      else it_eg->second.reset();
    }
  }

}
DirectionMap LEDPanel::initDirMap(){
  DirectionMap dir_map;
  //Fill all directions with pointers
  //In total there are 7 directions, where 0 is center, -3 is far left and +3 is far right
  for(int i = -3; i < 4; i++)
    dir_map[i] = make_pair(make_shared<BMP>(LED_COLS, LED_ROWS),make_shared<GIF>(LED_COLS,LED_ROWS,1));
  return dir_map;
}

ExprGifMap LEDPanel::initExprGifMap(){
  ExprGifMap eg_map;
  for (auto it = expr_str_map_.begin(); it != expr_str_map_.end(); it++ )
  {
    eg_map[it->first] = make_shared<GIF>(LED_COLS,LED_ROWS,1);
  }
  return eg_map;
}
void LEDPanel::timerGifPlayCallback(const ros::TimerEvent &evt){

  //True if the animation was not yet cycled through by defined count
  if(gif_opts_.cycles_completed < gif_opts_.cycles){

    //True if the GIF is not played completely yet
    if(gif_opts_.current_frame < gif_opts_.gif->getNumOfFrames()){
      writeToMatrix(gif_opts_.gif->array(gif_opts_.current_frame));
      gif_opts_.current_frame++;
    }
    else{
      gif_opts_.cycles_completed++;
      gif_opts_.current_frame = 0;
    }
  }
  else{
    gif_opts_.is_playing = false;
    stopGifAndShowBMP(expr_map_[LEDEx::NEUTRAL][eye_angle_opts_.num].first);
  }
}

void LEDPanel::timerBlinkCallback(const ros::TimerEvent &evt){
  //Play the Blinking animation
  if(current_expression_ == LEDEx::NEUTRAL && expressionIsOver()){
    //playGif(expr_map_[LEDEx::NEUTRAL][eye_angle_opts_.num].second,1);

    gif_play_queue_.push( [=] { playGif(expr_map_[LEDEx::NEUTRAL][eye_angle_opts_.num].second,1); } );
  }
}

void LEDPanel::timerMainCallback(const ros::TimerEvent &evt){

  if((ros::Time::now() - eye_angle_opts_.time_set).toSec() > MAX_DUR_PERSON){
    eye_angle_opts_.num = 0;
  }

  //True if the expression duration run out, reset to neutral expression
  if(expressionIsOver() && current_expression_ != LEDEx::NEUTRAL){
    current_expression_ = LEDEx::NEUTRAL;

    //True if no expression exists for the defined angle
    if(expr_map_[current_expression_][eye_angle_opts_.num].first == NULL){
      eye_angle_opts_.num = 0;
    }
    bmp_holder_.set_value(expr_map_[current_expression_][eye_angle_opts_.num].first);
  }


  if(!gif_play_queue_.empty() && !gif_opts_.is_playing){
    auto play_gif = gif_play_queue_.front();
    ROS_DEBUG_STREAM("Playing GIF from queue");
    play_gif();
    gif_play_queue_.pop();
  }
  else if(!gif_opts_.is_playing && bmp_holder_.has_new_value()){
    //If no GIF is played, show the BMP that was last set into the holder
    ROS_DEBUG_STREAM("Writing BMP to matrix");
    shared_ptr<BMP> bmp;
    bmp_holder_.get_value(bmp);
    writeToMatrix(*bmp);
  }
}

std::string LEDPanel::getFileNameFromExpression(int expression, int direction, imgtype type){

  std::string filename = expr_str_map_[expression];

  //append the number which represents the direction, _0 as behind filename represents -3 as direction, _1 represents -2...
  if(direction < 4 && direction > -4)
    filename += "_" + to_string(direction + 4);
  else
    filename +="_4";
  if(type == tGIF) filename += ".gif";
  else filename += ".bmp";

  return filename;
}

std::string LEDPanel::getTransitionFileNameFromExpressions(int expression_1, int expression_2,int direction){

  std::string filename = expr_str_map_[expression_1] + "_to_" + expr_str_map_[expression_2];
  if(direction < 4 && direction > -4)
    filename += "_" + to_string(direction + 4);
  else
    filename +="_4";

  filename += ".gif";
  return filename;

}
int LEDPanel::getNumFromAngle(const double &angle){
  double angle_norm = (angle/M_PI * 180.)/MAX_EYE_ANGLE;
  if(angle_norm >= 1 || angle_norm <= -1) return 0;

  return (int)(angle_norm * 4);
}

bool LEDPanel::angleInRange(const double &angle){
  double angle_deg = angle/M_PI * 180.;
  return angle_deg < MAX_EYE_ANGLE && angle_deg > -MAX_EYE_ANGLE;
}

void LEDPanel::stopGifAndShowBMP(shared_ptr<BMP> bmp){
  tim_gif_play_.stop();
  bmp_holder_.set_value(expr_map_[LEDEx::NEUTRAL][eye_angle_opts_.num].first);
}

bool LEDPanel::servSetLEDExpressionCallback(cmr_msgs::SetLEDExpression::Request &req, cmr_msgs::SetLEDExpression::Response &res){

  auto gif = expr_map_[req.expression.expression][0].second;
  auto bmp = expr_map_[req.expression.expression][0].first;

  //NEW
  if(req.expression.expression == LEDEx::NEUTRAL){
    current_expression_ = LEDEx::NEUTRAL;
    gif_opts_.is_playing = false;
    stopGifAndShowBMP(expr_map_[LEDEx::NEUTRAL][0].first);
  }
  //NEW

  //Priotize the GIF over the BMP
  if(gif != NULL){
    ROS_DEBUG_STREAM("Service setting GIF to queue");
    queueExpression(req.expression.expression,req.duration);
  }
  else if(bmp != NULL){
    ROS_DEBUG_STREAM("Service setting BMP to holder");
    bmp_holder_.set_value(bmp);
    current_expression_ = req.expression.expression;
  }
  else
    return false;

  time_expression_reset_ = ros::Time::now() + ros::Duration(req.duration);

  return true;

}

void LEDPanel::subPersonsCallback(const cmr_msgs::EyeFollowerPersonsConstPtr &msg){

  //Only change angle every x seconds
  if(!msg->tracks.empty() && (ros::Time::now() - eye_angle_opts_.time_set).toSec() > 1.2){

    //Get iterator to closest person
    auto min_it = std::min_element(msg->tracks.begin(), msg->tracks.end(),
                                   [](const cmr_msgs::EyeFollowerPerson &person1, const cmr_msgs::EyeFollowerPerson &person2){
      return person1.distance < person2.distance;
    });

    if(min_it->distance < MAX_EYE_DIST
       && angleInRange(min_it->angle)
       && eye_angle_opts_.num != getNumFromAngle(min_it->angle)
       && msg->tracks.size() != num_detections_){
      //Only change angle if number of detections changed
      num_detections_ = msg->tracks.size();
      eye_angle_opts_.num = getNumFromAngle(min_it->angle);
      eye_angle_opts_.time_set = ros::Time::now();

      //True if no expression exists for the defined angle
      if(expr_map_[current_expression_][eye_angle_opts_.num].first == NULL){
        eye_angle_opts_.num = 0;
      }

      bmp_holder_.set_value(expr_map_[current_expression_][eye_angle_opts_.num].first);

    }

  }
  else{
    num_detections_ = 0;
  }

}
//void sigIntHandler(int sig){
// ros::shutdown();
//}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "led_panel");

  ros::NodeHandle node_handle;
  LEDPanel LEDPanel(node_handle);

  ROS_INFO("LED Panel started");
  ros::spin();
  return 0;
}
