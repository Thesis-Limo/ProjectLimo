# How to use your pc as monitor for the Limo
* Make sure the limo is turned on 
* Set your IP addres in the limo's file named /etc/hosts
* Set the IP address of the limo in your pc's file /etc/hosts
* At last in every terminal on the external pc you need to add the following: export ROS_MASTER_URI=http://<< hostname ip >>:11311

# How to run pipeline in Limo
On the limo, just run a terminal, everthing should be configured, there is a script running roscore. So don't worry about that.
After sync with the limo, with the github,
Run the file dependencyLimo, here you first need to run the dependency if changed, then run the build. So then the build is finished and start running the pipeline

# Adding dependency from apt-get install
* add to the dockerfile
* add to the dependencyLimo file