import sys

def check_import(module_name):
    try:
        __import__(module_name)
        print(f"[OK] {module_name}")
        return True
    except ImportError as e:
        print(f"[FAIL] {module_name}: {e}")
        return False

def main():
    print("Verifying dependencies...")
    
    # Standard/PyPI dependencies
    dependencies = [
        "fastapi",
        "uvicorn",
        "jinja2",
        "requests",
        "bs4", # beautifulsoup4
        "lxml",
        "selenium",
        "webdriver_manager",
        "cv2", # opencv-python
        "numpy",
    ]
    
    all_passed = True
    
    print("\n--- PyPI Dependencies ---")
    for dep in dependencies:
        if not check_import(dep):
            all_passed = False
            
    if all_passed:
        print("\nAll dependencies verified successfully!")
        sys.exit(0)
    else:
        print("\nSome dependencies failed to import.")
        sys.exit(1)

if __name__ == "__main__":
    main()
