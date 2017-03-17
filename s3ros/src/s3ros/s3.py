from aws_msgs.msg import Upload
import boto3
import roslib
from Queue import Queue
import rospy
from std_msgs.msg import Bool
import sys


class s3ros:
    
    def __init__(self):
        """Init and run main loop
        """
        self.uploadsPaused_ = True
        self.uploadQueue_ = Queue()

        client = boto3.client('s3')

        # Pubs, Subs & Srvs
        rospy.Subscriber("uploadLocalFile", Upload, self.localUploadCB)
        rospy.Subscriber("pauseUploads", Bool, self.pauseUploads)

        # Main loop monitors upload queue and uploads when necessary
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not uploadsPaused_ and not uploadQueue_.empty():
                toUpload = uploadQueue_.get()
                try:
                    rsp = client.upload_file(*toUpload)
                except boto3.exceptions.S3UploadFailedError:
                    rospy.logerror("Could not uplaod {0} to bucket {1}/{2}".format(myfun(*toUpload)))

            r.sleep()
            rospy.spinOnce()

        if not uploadQueue_.empty():
            rospy.logwarn("Upload queue not empty")
            
                
    def pauseUploadsCB(self, boolMsg):
        """Callback for toggling uploads
        """
        uploadsPaused_ = boolMsg.data

        
    def localUploadCB(self, upMsg):
        """Callback for adding a local file to the upload queue
        """
        uploadQueue_.put((upMsg.filepath, upMsg.bucket, upMsg.key))
        

