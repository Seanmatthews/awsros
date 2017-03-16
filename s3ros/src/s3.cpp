#include "s3.hpp"

using namespace Aws::S3;
using namespace Aws::Utils;
using namespace std;

namespace awsros
{

    S3::S3(ros::NodeHandle nh) :
        uploadsPaused_(true),
        nh_(nh)
    {

    }

    S3::~S3()
    {

    }


    // Check that the system credentials are valid
    bool S3::validCredentials(string& error)
    {
        // I'm not sure a better way to do this than to make a request
        auto result = client_.ListBuckets();
        if (!result.IsSuccess())
        {
            error = result.GetError().GetExceptionName()
                + ": " + result.GetError().GetMessage();
            return false;
        }
        return true;
    }

    // Form client config from params
    // TODO region, 
    ClientConfiguration S3::getClientConfig()
    {
        ClientConfiguration config;
        int readWriteLimit;
        
        nh_.getParam("config/connectTimeoutMs", config.connectTimeoutMs, 30000);
        nh_.getParam("config/requestTimeoutMs", config.requestTimeoutMs, 30000);        
        nh_.getParam("config/readWriteLimit", readWriteLimit, 20000);
        
        config.scheme = Scheme::HTTPS;
        auto limiter =
            Aws::MakeShared<RateLimits::DefaultRateLimiter<> >(ALLOCATION_TAG, readWriteLimit);
        return config;
    }

    // Upload a local file to a bucket only if that bucket exists
    bool S3::uploadToBucket(string bucket, string key, string filepath, string& error)
    {
        auto result = client_.ListBuckets();
        if (!result.IsSuccess())
        {
            error = result.GetError().GetExceptionName()
                + ": " + result.GetError().GetMessage();            
            return false;
        }

        Aws::Vector<Aws::S3::Model::Bucket> buckets = result.GetResult().GetBuckets();
        for (auto& b : buckets)
        {
            if (bucket == b.GetName())
            {
                Model::PutObjectRequest putReq;
                putReq.WithBucket(bucket).WithKey(key);
                auto data = Aws::MakeShared<Aws::FStream>("PutObjectInputStream",
                                                          filepath.c_str(),
                                                          ios_base::in | ios_base::bin);
                putReq.SetBody(data);
                auto putRes = client_.PutObject(putReq);

                if (!putRes.IsSuccess)
                {
                    error = putRes.GetError().GetExceptionName()
                        + ": " + putRes.GetError().GetMessage();
                    return false;
                }
                
                return true;
            }
        }
        return false;
    }
    
    // Setup, then monitor the paused state and handle upload queue
    void S3::run()
    {
        // AWS setup
        ClientConfiguration config = getClientConfig();
        client_ = Aws::MakeShared<S3Client>(ALLOCATION_TAG, config);

        string error;
        if (!validCredentials(error))
        {
            ROS_ERROR_STREAM(error);
            ROS_FATAL("AWS credentials not valid. Exiting..");
            return;
        }

        // Pubs, Subs & Srvs
        localUploadSub_ = nh_.subscribe("uploadLocalFile", 10, &S3::localUploadCB, this);
        uploadToggleSub_ = nh_.subscribe("pauseUploads", 1, &S3::uploadToggleCB, this);
        
        while (ros::ok())
        {
            if (!uploadsPaused_ && !uploadQueue_.empty())
            {
                auto toUpload = uploadQueue_.front();
                uploadQueue_.pop_front();
                string bucketName = get<0>(toUpload);
                string key = get<1>(toUpload);
                string filename = get<2>(toUpload);

                if (!uploadFileToBucket(bucketName, filename))
                {
                    ROS_ERROR_STREAM("Could not upload " << filename << " to " << bucketName);
                }
            }
            ros::Duration(0.2).sleep();
            ros::spinOnce();
        }
    }

    // Add a local file to upload queue
    void S3::localUploadCB(const awsros::UploadConstPtr& msg)
    {
        uploadQueue_.push_back(make_tuple(msg->bucket, msg->key, msg->filepath));
    }

    // Toggle whether we're uploading files
    void S3::uploadToggleCB(const std_msgs::BoolConstPtr& msg)
    {
        uploadsPaused_ = msg->data;
    }

}
