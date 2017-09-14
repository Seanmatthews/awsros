# awsros
AWS with a ROS interface

## Why?
I teetered between making this ROS node and not, due to an overall lack of value-add.
Ultimately, I decided to create the node in order to place AWS functions into the 
framework of ROS's asynchronous architecture. Specifically, I found myself thinking
of use cases where some node running on robot records video but wishes to delay
upload until it returns to a base station. Or similarly, if several robots signal
that their locally-recorded videos are ready for upload, this node can queue them
and then wait for a signal that a particular robot has returned with its video data.
Presumably, the robots are connected to a network, but we may not wish to clog that
network with large amounts of video data while we send higher-priority control commands.
I'm certain you can find alternative solutions for these use cases, but I like this one.

## Packages

### s3ros
Provides basic upload functionality to S3 buckets.