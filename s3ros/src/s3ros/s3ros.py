from aws_msgs.msg import Upload
import boto3
from collections import deque
import os
import roslib
import rospy
from std_msgs.msg import Bool
import sys


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

        client = boto3.client('s3')

        # Pubs, Subs & Srvs
        rospy.Subscriber("s3ros/uploadLocalFile", Upload, self.localUploadCB)
        rospy.Subscriber("s3ros/pauseUploads", Bool, self.pauseUploadsCB)

        # Main loop monitors upload queue and uploads when necessary
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.uploadsPaused_ and len(self.uploadQueue_):
                toUpload = self.uploadQueue_.popleft()
                if not os.path.isfile(toUpload):
                    continue
                
                try:
                    rospy.loginfo("Attempting to upload {} to {}/{}".format(*toUpload))
                    rsp = client.upload_file(toUpload[0], toUpload[1], toUpload[2])

                except boto3.exceptions.S3UploadFailedError:
                    rospy.logerror("Could not uplaod {0} to bucket {1}/{2}".format(myfun(*toUpload)))

            r.sleep()

        if len(self.uploadQueue_):
            rospy.logwarn("Upload queue not empty")
            
                
    def pauseUploadsCB(self, boolMsg):
        """Callback for toggling uploads
        """
        rospy.loginfo("Uploads paused? {}".format(boolMsg.data == True))
        self.uploadsPaused_ = boolMsg.data

        
    def localUploadCB(self, upMsg):
        """Callback for adding a local file to the upload queue.
        """
        rospy.loginfo("Adding {} to the upload queue".format(upMsg.filepath))
        self.uploadQueue_.append((upMsg.filepath, upMsg.bucket, upMsg.key))

        # Automatically remove duplicates
        self.uploadQueue_ = deque(set(self.uploadQueue_))

