#include <cstdlib>
#include "comb/comb.h"
// #include "comb/comb_psesudo.h"
#include <std_msgs/Float32.h>
#include "comb/utils.h"

#include <glog/logging.h>
/*
The intensity of the filter function was computed in log intensity state.
There is a particular function to convert that back to the image file.
*/


enum {GAUSSIAN, BILATERAL};

namespace comb{

    Comb_filter::Comb_filter(ros::NodeHandle &nh, ros::NodeHandle nh_private){

        constexpr int INTENSITY_ESTIMATE_PUB_QUEUE_SIZE = 1;
        constexpr double EVENT_RETENTION_DURATION = 30; // seconds. Used for calibrating contrast thresholds.

        std::string working_dir;
        std::string save_dir;

        nh_private.getParam("publish_framerate", publish_framerate_);
        nh_private.getParam("save_dir", save_dir);
        nh_private.getParam("working_dir", working_dir);

        VLOG(1) << "Construction framerate: " << publish_framerate_;

        if (save_dir.empty()){

            save_images_ = false;
        }
        else{

            save_images_ = true;
            save_dir_ = comb::utils::fullpath(working_dir, save_dir);
            if (save_dir_.back() != '/'){

                save_dir_.append("/");
            }

            const int dir_err = system((std::string("mkdir -p ") + save_dir_).c_str());
            if (-1 == dir_err){
                LOG(ERROR) << "Error crearing save directory!";
                return;
            }

            VLOG(1) << "Save image to: " << save_dir_;
        }

        // setup publishers
        image_transport::ImageTransport it_(nh_);
        intensity_estimate_pub_ = it_.advertise("comb/intensity_estimate", INTENSITY_ESTIMATE_PUB_QUEUE_SIZE);

        // flags and counters
        initialised_ = false;

        // low-pass parameter to reach 95% of a constant signal in EVENT_RETENTION_DURATION seconds.
        event_count_cutoff_frequency_ = -std::log(1 - 0.95) / EVENT_RETENTION_DURATION; // rad/s.

        contrast_threshold_on_adaptive_ = 0.1; // fixed by convention
        contrast_threshold_off_adaptive_ = -0.1;

        t_next_publish_ = 0.0;
        t_next_recalibrate_contrast_thresholds_ = 0.0;
        t_next_log_intensity_update_ = 0.0;

    }

    Comb_filter::~Comb_filter(){

        intensity_estimate_pub_.shutdown();

        // free the buffer space before exit
        delete[] ring_buffer1_;
        delete[] ring_buffer2_;
    }

    void Comb_filter::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg){
        // initialise image states
        uint32_t msg_height = msg->height;
        uint32_t msg_width = msg->width;

        if(msg_height == 0 || msg_width == 0){
            if(user_defined_size_){
                msg_height = height_user_defined_;
                msg_width = width_user_defined_;
            }
            else{
                user_size_input();
                msg_height = height_user_defined_;
                msg_width = width_user_defined_;
            }
        }

        if (!initialised_){

            // width and height
            initialise_image_states(msg_height, msg_width);
        }

        if (msg->events.size() > 0){
            // testing
            // VLOG(1) << msg->events.size();
            for (int i = 0; i < msg->events.size(); i++){

                const int x = msg->events[i].x;
                const int y = msg->events[i].y;

                // VLOG(1) << "x = " << x << ", y = "<< y;
                // VLOG(1) <<  msg_height << " " << msg_width;

                // msg->width
                if (x > 0 && x < msg_width && y > 0 && y < msg_height){

                    const double ts = msg->events[i].ts.toSec();
                    // VLOG(1) << ts;
                    const bool polarity = msg->events[i].polarity;

                    if (adaptive_contrast_threshold_){

                        update_leaky_event_count(ts, x, y, polarity);
                    }

                    // integral tracking
                    integral_tracking(x, y, polarity);

                    while(ts > t_next_store_){
                        if(t_next_store_ == 0){
                            t_next_store_ = ts;
                        } 

                        cv::Mat x0_e = cv::Mat::zeros(msg_height, msg_width,CV_64FC1); // exponentional version of x0_
                        exp_of_log(x0_e);

                        grab_delay(x_d1_, int(d1_ * mtr_), 1);
                        grab_delay(x_d2_, int(d2_ * mtr_), 1);
                        grab_delay(x_d12_, int(d12_ * mtr_), 1);

                        grab_delay(y_d1_, int(d1_ * mtr_), 2);
                        grab_delay(y_d2_, int(d2_ * mtr_), 2);
                        grab_delay(y_d12_, int(d12_ * mtr_), 2);
                        // calculate new y0_
                        switch(filtering_method_){
                            case 1:{
                                y0_ = x0_e; // direct integration
                                break;
                            }
                            case 2:{
                                y0_ = x0_e - x_d1_; // simple version comb (no dc)
                                break;
                            }
                            default:{
                                y0_ = x0_e - x_d1_ - rho2_ * x_d2_ + rho2_ * x_d12_ + rho1_ * y_d1_ + y_d2_ - rho1_ * y_d12_;
                            }

                        }
                        
                       
                        
                        // FIXME
                        // std::cout << x_d1_ << std::endl;
                        store2buffer(x0_e, y0_);

                        t_next_store_ += 1/mtr_;
                    }

                    if (publish_framerate_ > 0 && ts >= t_next_publish_){
                        
                        // VLOG(1) << "New publish";
                        publish_intensity_estimate(msg->events[i].ts);
                        t_next_publish_ = ts + 1 / publish_framerate_;
                    }
                }
            }
            const double ts = msg->events.back().ts.toSec();

            if(adaptive_contrast_threshold_ && (ts > t_next_recalibrate_contrast_thresholds_)){
                constexpr double contrast_threshold_recalibration_frequency = 20.0; // Hz
                recalibrate_contrast_thresholds(ts);
                t_next_recalibrate_contrast_thresholds_ = ts + 1/contrast_threshold_recalibration_frequency;
            }
        }
    }

    // passing by const reference
    void Comb_filter::initialise_image_states(const uint32_t &rows, const uint32_t &columns){

        log_intensity_state_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        leaky_event_count_on_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        leaky_event_count_off_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        ts_array_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        ts_array_on_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        ts_array_off_ = cv::Mat::zeros(rows, columns, CV_64FC1);

        // publish_framerate_ = 100;
        t_next_publish_ = 0.0;
        t_next_recalibrate_contrast_thresholds_ = 0.0;
        t_next_log_intensity_update_ = 0.0;

        // come from dynamic reconfiguration
        cutoff_frequency_global_ = 0.06 * 2 * M_PI;
        cutoff_frequency_per_event_component_ = 0.06;
        contrast_threshold_on_user_defined_ = 0.1;
        contrast_threshold_off_user_defined_ = -0.1;
        intensity_min_user_defined_ = -0.5;
        intensity_max_user_defined_ = 1.5;
        adaptive_contrast_threshold_ = false;
        spatial_filter_sigma_ = 0;
        spatial_smoothing_method_ = 0;
        adaptive_dynamic_range_ = false;
        color_image_ = false;
        // user_defined_size_ = false;

        // image size
        // std::cin >> height_user_defined_;
        // std::cin >> width_user_defined_;

        // delayed version of integrated events
        x0_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        x_d1_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        x_d2_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        x_d12_ = cv::Mat::zeros(rows, columns, CV_64FC1);

        // delayed version of output signal
        y0_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        y_d1_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        y_d2_ = cv::Mat::zeros(rows, columns, CV_64FC1);
        y_d12_ = cv::Mat::zeros(rows, columns, CV_64FC1);

        // time delay
        double base_freq;
        std::cout << "Enter base frequency: " << std::endl;
        std::cin >> base_freq;
        std::cout << "Filtering method (1:direct integration, 2:comb, 3:improved comb):" << std::endl;
        std::cin >> filtering_method_;
        
        d1_ = 1/base_freq;
        d2_ = d1_/10;
        d12_ = d1_ + d2_;
        
        // delay gain
        rho1_ = 0.99;  // distortion reduce factor
        rho2_ = 0.999; // compensate factor

        // FIXME
        roll_back_ = false;

        initialise_buffer(rows, columns);

        initialised_ = true;
        
        VLOG(2) << "Initialised!";
    }

// insider whether to put buffer initialisation to the start of the program
    void Comb_filter::initialise_buffer(const uint32_t &rows, const uint32_t &columns){

        // minimum time resolution
        mtr_ = 1e5; // NOTE: this should be 1e-5, but due to the accuracy of the floating point number we use positive here!
        t_next_store_ = 0.0;

        // FIXME
        std::cout << "tao1:" << d1_ << " tao2:" << d2_ << " tao12:" << d12_ << std::endl;
        std::cout << 1/mtr_ << std::endl;
        buffer_length_ = int(d12_ * mtr_ + 1);
        std::cout << "Buffer length:" <<buffer_length_ << std::endl;
        buffer_index_ = 0;

        // int sizes[] = {rows, columns, buffer_length_};
        // ring_buffer1_ = new cv::Mat(3, sizes, CV_64FC1, cv::Scalar(0));
        // ring_buffer2_ = new cv::Mat(3, sizes, CV_64FC1, cv::Scalar(0));

        ring_buffer1_ = new cv::Mat[buffer_length_];
        ring_buffer2_ = new cv::Mat[buffer_length_];

        // zero initialisation for all buffers
        for (int i = 0; i < buffer_length_; i++){

            cv::Mat my_zero = cv::Mat::zeros(rows, columns, CV_64FC1);
            my_zero.copyTo(ring_buffer1_[i]);
            my_zero.copyTo(ring_buffer2_[i]);
        }

        // FIXME
        // std::cout << "Size of ring buffer: " << sizeof(ring_buffer1_) << std::endl;
    }

    // tracking the integral to the current state
    void Comb_filter::integral_tracking(const int x, const int y, const bool polarity){

        double c_times_p;
        if (adaptive_contrast_threshold_){

            c_times_p = (polarity) ? contrast_threshold_on_adaptive_ : contrast_threshold_off_adaptive_;
        }
        else{

            c_times_p = (polarity) ? contrast_threshold_on_user_defined_ : contrast_threshold_off_user_defined_;
        }
        x0_.at<double>(y, x) = x0_.at<double>(y, x) + c_times_p;
    }

    void Comb_filter::store2buffer(const cv::Mat& figx, const cv::Mat& figy){

        // buffer index of input and output is the same
        figx.copyTo(ring_buffer1_[buffer_index_]);
        figy.copyTo(ring_buffer2_[buffer_index_]);

        buffer_index_ ++;
        // ring buffer
        if (buffer_index_ == buffer_length_){

            buffer_index_ = 0;
        }

        // after the storing, the index is one element ahead
    }

    void Comb_filter::update_leaky_event_count(const double& ts, const int& x, const int& y, const bool& polarity){
        if (polarity){
            // positive ON event
            const double delta_t = (ts - ts_array_on_.at<double>(y, x));

            if (delta_t >= 0){

                leaky_event_count_on_.at<double>(y, x) = std::exp(-event_count_cutoff_frequency_ * delta_t)
                * leaky_event_count_on_.at<double>(y, x) + 1;
                ts_array_on_.at<double>(y, x) = ts;
            }
        }
        else{
        // negative OFF event
            const double delta_t = (ts - ts_array_off_.at<double>(y, x));
            if (delta_t >= 0){

                leaky_event_count_off_.at<double>(y, x) = std::exp(-event_count_cutoff_frequency_ * delta_t)
                * leaky_event_count_off_.at<double>(y, x) + 1;
                ts_array_off_.at<double>(y, x) = ts;
            }
        }
    }

    void Comb_filter::recalibrate_contrast_thresholds(const double& ts){
  
        constexpr double EVENT_DENSITY_MIN = 5e6;
        //first do global update
        cv::Mat decay_factor_on;
        cv::Mat decay_factor_off;
        cv::exp(-event_count_cutoff_frequency_ * (ts - ts_array_on_), decay_factor_on);
        cv::exp(-event_count_cutoff_frequency_ * (ts - ts_array_off_), decay_factor_off);

        leaky_event_count_on_ = leaky_event_count_on_.mul(decay_factor_on);
        leaky_event_count_off_ = leaky_event_count_off_.mul(decay_factor_off);

        ts_array_on_.setTo(ts);
        ts_array_off_.setTo(ts);

        const double sum_on = cv::sum(leaky_event_count_on_)[0];
        const double sum_off = cv::sum(leaky_event_count_off_)[0];
        
        if (sum_on + sum_off > EVENT_DENSITY_MIN){
        
            contrast_threshold_off_adaptive_ = -sum_on / (sum_off + 1e-10) * contrast_threshold_on_adaptive_; // re-calibrate contrast thresholds
        }
    }

    void Comb_filter::grab_delay(cv::Mat& sel, const int i1, const int which_buffer){

        int index = buffer_index_ - i1;
        // cv::Mat sel;

        if(index < 0){
            index = buffer_length_ + index;
        }
        // FIXME
        std::cout << index << std::endl;
        // if(index == 4 && roll_back_){
        //     std::cout << x_d1_ << std::endl;
        // }
        // else if(index == 4){
        //     roll_back_ = true;
        // }

        if(which_buffer == 1){
            ring_buffer1_[index].copyTo(sel);
        }
        else{
            ring_buffer2_[index].copyTo(sel);
        }

        // return sel;
    }

    void Comb_filter::publish_intensity_estimate(const ros::Time &timestamp){

        cv::Mat display_image;
        cv_bridge::CvImage cv_image;

        // convert_log_intensity_state_to_display_image(display_image, timestamp.toSec());
        // output_regulator(display_image, timestamp.toSec());
        y0_.convertTo(display_image, CV_8UC1, 255.0);

        if (color_image_){

            cv::Mat color_display_image;
            cv::cvtColor(display_image, color_display_image, CV_BayerBG2BGR_EA);
            display_image = color_display_image;
            cv_image.encoding = "bgr8";
        }
        else{

            cv_image.encoding = "mono8";
        }

        if (spatial_filter_sigma_ > 0){

            cv::Mat filtered_display_image;
            if (spatial_smoothing_method_ == GAUSSIAN){

                cv::GaussianBlur(display_image, filtered_display_image, cv::Size(5, 5), spatial_filter_sigma_, spatial_filter_sigma_);
            }
            else if (spatial_smoothing_method_ == BILATERAL){

                const double bilateral_sigma = spatial_filter_sigma_ * 25;
                cv::bilateralFilter(display_image, filtered_display_image, 5, bilateral_sigma, bilateral_sigma);
            }
            display_image = filtered_display_image; // data is not copied
        }

        cv_image.image = display_image;
        cv_image.header.stamp = timestamp;
        intensity_estimate_pub_.publish(cv_image.toImageMsg());
        VLOG(1) << "Publish image message";

        if (save_images_){

            static int image_counter = 0;
            std::string save_path = save_dir_ + "image" + std::to_string(image_counter) + ".png";
            cv::imwrite(save_path, display_image);
            image_counter++;
        }
    }

    void Comb_filter::exp_of_log(cv::Mat& converted_image){

        double LOG_INTENSITY_OFFSET = std::log(1.5); // chosen because standard APS frames range from [1, 2].

        x0_.copyTo(converted_image);
        converted_image += LOG_INTENSITY_OFFSET;
        cv::exp(converted_image, converted_image);
        converted_image -= 1;

    }

    void Comb_filter::output_regulator(cv::Mat& image_out, const double &ts){

        cv::Mat image;

        y0_.copyTo(image);

        static double intensity_lower_bound = intensity_min_user_defined_;
        static double intensity_upper_bound = intensity_max_user_defined_;
        static double t_last = 0.0;
        const double delta_t = ts - t_last;

        if(delta_t >= 0){

            intensity_lower_bound = intensity_lower_bound;
            intensity_upper_bound = intensity_upper_bound;
        }

        const double intensity_range = intensity_upper_bound - intensity_lower_bound;
        image -= intensity_lower_bound;
        
        image.convertTo(image_out, CV_8UC1, 255.0 / intensity_range);
        t_last = ts;

    }

    void Comb_filter::user_size_input(){

        std::cout << "Enter user defined height and width:" << std::endl;
        std::cout << "Enter height: ";
        std::cin >> height_user_defined_;
        std::cout << "Enter width: ";
        std::cin >> width_user_defined_;

        user_defined_size_ = true;
    }

}