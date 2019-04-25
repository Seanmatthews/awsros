import os
from collections import deque

import boto3
import rospy
from aws_msgs.msg import Upload
from boto3.s3.transfer import TransferConfig
from std_msgs.msg import Bool

MULTIPART_THRESHOLD = 5 * 1024 * 1024


class s3ros:
    """
    Simple class for interfacing S3 functionality. 
    Currently limited to upload.
    """

    def __init__(self):
        """Init and run main loop
        """
        self.uploadsPaused_ = False
        self.uploadQueue_ = deque()

        # Get alternate endpoint for base station video offload
        endpoint = rospy.get_param('~s3_endpoint')
        if "" == endpoint:
            endpoint = None
        else:
            rospy.loginfo("Using alternate S3 endpoint {}".format(endpoint))

        s3 = boto3.resource(
            service_name='s3',
            endpoint_url=endpoint
        )

        upload_config = TransferConfig(multipart_threshold=MULTIPART_THRESHOLD)
        client = s3.meta.client

        # Pubs, Subs & Srvs
        rospy.Subscriber("s3ros/uploadLocalFile", Upload, self.localUploadCB)
        rospy.Subscriber("s3ros/pauseUploads", Bool, self.pauseUploadsCB)

        # Main loop monitors upload queue and uploads when necessary
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if len(self.uploadQueue_) > 0:
                toUpload = self.uploadQueue_.popleft()

                if not os.path.isfile(toUpload[0]):
                    rospy.logwarn("{} is not a file".format(toUpload[0]))
                    continue
                
                try:
                    rospy.loginfo("Attempting to upload {} to {}/{}/{}".format(toUpload[0], endpoint, toUpload[1], toUpload[2]))
                    rsp = client.upload_file(toUpload[0], toUpload[1], toUpload[2], Config=upload_config)
                    rospy.loginfo("Upload succeeded")

                except boto3.exceptions.S3UploadFailedError as e:
                    rospy.logerr("Could not upload {0} to bucket {1}/{2}".format(*toUpload))
                    rospy.logerr(e)
                except Exception as e:
                    rospy.logerr(e)

            r.sleep()

        if len(self.uploadQueue_) > 0:
            rospy.logwarn("Upload queue not empty")
            
                
    def pauseUploadsCB(self, boolMsg):
        """Callback for toggling uploads
        """
        rospy.loginfo("Uploads paused? {}".format(boolMsg.data is True))
        self.uploadsPaused_ = boolMsg.data

        
    def localUploadCB(self, upMsg):
        """Callback for adding a local file to the upload queue.
        """
        rospy.loginfo("Adding {} to the upload queue".format(upMsg.filepath))
        self.uploadQueue_.append((upMsg.filepath, upMsg.bucket, upMsg.key))

        # Automatically remove duplicates
        self.uploadQueue_ = deque(set(self.uploadQueue_))
