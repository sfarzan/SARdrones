# parent_script.py
import subprocess

# Define the command to run the child script
command = "python child_process.py arg1 arg2"

# Run the command as a subprocess
process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# Communicate with the process to get the output and errors
stdout, stderr = process.communicate()

# Print the output and errors
print("Output:\n", stdout)
print("Errors:\n", stderr)

# Print the return code
print("Return code:", process.returncode)
