limo_ip() {
    local ip_address="$1"
    local existing_name="agilex"
    
    export ROS_MASTER_URI="http://$ip_address:11311"
    local hostname_ip=$(hostname -I | awk '{print $1}')
    export ROS_IP="$hostname_ip"
    
    # Update /etc/hosts file
    echo "$ip_address $existing_name" | sudo tee -a /etc/hosts > /dev/null
}

build_planner() {
    cd ROS/src/limo_motion_planner/src && python3 setup.py build install
    cd ../../../../
}