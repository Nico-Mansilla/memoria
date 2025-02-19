import subprocess

def run_script(script_path):
    try:
        result = subprocess.run(['sudo', 'run', '-n', script_path], check=True, capture_output=True, text=True)
        print(f"Output of {script_path}:\n{result.stdout}")
    except subprocess.CalledProcessError as e:
        print(f"Error running {script_path}:\n{e.stderr}")

if __name__ == "__main__":
    run_script('/home/nicolas/Desktop/Serial/config_usb0.py')
    run_script('/home/nicolas/Desktop/Serial/config_usb1.py')