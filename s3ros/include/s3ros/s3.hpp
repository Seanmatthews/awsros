#ifndef S3_HPP_
#define S3_HPP_

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>

#include <deque>

#include "awsros/Upload.h"

namespace awsros
{
    class S3
    {
    public:
        S3(ros::NodeHandle nh);
        ~S3();

        void run();
        
    private:
        bool validCredentials(std::string& error);
        void localUploadCB(const awsros::UploadConstPtr& msg);
        void uploadToggleCB(const std_msgs::BoolConstPtr& msg);        

        Aws::S3::S3Client client_;
        bool uploadsPaused_;
        std::deque<std::string> uploadQueue_;

        ros::NodeHandle nh_;
        ros::Subscriber localUploadSub_;
        ros::Subscriber uploadToggleSub_;
    };
    
}

#endif
