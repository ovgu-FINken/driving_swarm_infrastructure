from robot_upstart.job import Job

if __name__ == '__main__':
    print("installing driving_swarm_core and driving_swarm_demo systemd service")
    job = Job('DrivingSwarmCore', interface='wlan0', user='turtle', description='driving_swarm_core')
    job.add(package='driving_swarm_core', filename='driving_swarm_core.launch.py')
    job.install()
    
    job = Job('DrivingSwarmDemo', interface='wlan0', user='turtle', description='driving_swarm_demo')
    job.add(package='driving_swarm_core', filename='driving_swarm_demo.launch.py')
    job.install()