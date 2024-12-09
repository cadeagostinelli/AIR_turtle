import subprocess
import time

def start_processes():
    """Start tracker and controller as subprocesses."""
    processes = {}
    try:
        print("Starting Tracker...")
        processes["tracker"] = subprocess.Popen(["python", "tracker.py"])
        
        print("Starting Controller...")
        processes["controller"] = subprocess.Popen(["python", "controller.py"])
        
        return processes
    except Exception as e:
        print(f"Error starting processes: {e}")
        stop_processes(processes)
        return None

def stop_processes(processes):
    """Stop all running subprocesses."""
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
        print("All processes terminated. Goodbye!")

if __name__ == "__main__":
    main()
