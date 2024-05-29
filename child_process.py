# child_script.py
import sys

def main():
    print("Child script is running!")
    for arg in sys.argv[1:]:
        print(f"Argument: {arg}")

if __name__ == "__main__":
    main()