import subprocess
import time

def start_processes():
    processes = {}
    try:
        print("Starting Tracker...")
        processes["tracker"] = subprocess.Popen(["python", "tracker.py"])
        
        print("Starting Controller...")
        processes["controller"] = subprocess.Popen(["python", "controller.py"])
        
        print("Starting Turtlesim...")
        # Start turtlesim node
        processes["turtlesim"] = subprocess.Popen(["ros2", "run", "turtlesim", "turtlesim_node"])
        
        return processes
    except Exception as e:
        print(f"Error starting processes: {e}")
        stop_processes(processes)
        return None

def stop_processes(processes):
    for name, process in processes.items():
        if process is not None and process.poll() is None:
            print(f"Terminating {name}...")
            process.terminate()
            process.wait()

def main():
    print("Initializing system...")
    processes = start_processes()
    if not processes:
        print("Failed to start processes. Exiting...")
        return

    try:
        print("System is running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1) 
    except KeyboardInterrupt:
        print("\nExiting program. Stopping all processes...")
    finally:
        stop_processes(processes)
        print("Goodbye!")

if __name__ == "__main__":
    main()
