# How to use your pc as monitor for the Limo
* Make sure the limo is turned on 
* Set your IP addres in the limo's file named /etc/hosts
* Set the IP address of the limo in your pc's file /etc/hosts
* for best practice, the following, in a file or in you ~/.bashrc file, on the limo and also on your pc:
    * export ROS_MASTER_URI=http://"ip of the limo":11311
    * export ROS_IP="ip of the device"

    =>this needs to be done in every terminal page


# How to run pipeline in Limo
After sync with the limo, with the github,
Run the file dependencyLimo, here you first need to run the dependency if changed, then run the build. So then the build is finished and start running the pipeline

# Adding dependency from apt-get install
* add to the dockerfile
* add to the dependencyLimo file