import os
import sys
import subprocess
import platform
import argparse
import shutil

SETUP_DIR = os.path.dirname(os.path.abspath(__file__))


def find_python_version():
    for executable in ["python3.11", "python3", "python"]:
        try:
            version_output = subprocess.check_output([executable, "--version"], stderr=subprocess.STDOUT)
            version_str = version_output.decode("utf-8").strip().split()[1]
            version_tuple = tuple(map(int, version_str.split('.')))
            if version_tuple[:2] == (3, 11):
                print(f"Using Python executable: {executable} (version {version_str})")
                return executable
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue

    print("ERROR: Python 3.11 is required but was not found.")
    sys.exit(1)


def create_venv(python_executable, venv_name):
    try:
        subprocess.check_call([python_executable, "-m", "venv", venv_name])
        print(f"Virtual environment '{venv_name}' created successfully.")
    except subprocess.CalledProcessError:
        print(f"ERROR: Failed to create virtual environment '{venv_name}'.")
        sys.exit(1)


def install_requirements(venv_name, requirements_file):
    if platform.system() == "Windows":
        pip_path = os.path.join(venv_name, "Scripts", "pip")
    else:
        pip_path = os.path.join(venv_name, "bin", "pip")

    try:
        subprocess.check_call([pip_path, "install", "-r", requirements_file])
        print(f"Installed '{requirements_file}' into '{venv_name}'.")
    except subprocess.CalledProcessError:
        print(f"ERROR: Failed to install requirements from '{requirements_file}'.")
        sys.exit(1)


def add_project_to_path(venv_name, project_path):
    if platform.system() == "Windows":
        site_packages_dir = os.path.join(venv_name, "Lib", "site-packages")
    else:
        python_bin = os.path.join(venv_name, "bin", "python3")
        if not os.path.exists(python_bin):
            python_bin = os.path.join(venv_name, "bin", "python")

        try:
            version_output = subprocess.check_output([python_bin, "--version"], stderr=subprocess.STDOUT)
            version_str = version_output.decode("utf-8").strip().split()[1]
            version_tag = f"python{'.'.join(version_str.split('.')[:2])}"
        except Exception as e:
            print(f"ERROR: Failed to detect Python version in '{venv_name}': {e}")
            sys.exit(1)

        site_packages_dir = os.path.join(venv_name, "lib", version_tag, "site-packages")

    if not os.path.exists(site_packages_dir):
        print(f"ERROR: Could not find site-packages in '{venv_name}': {site_packages_dir}")
        sys.exit(1)

    pth_file_path = os.path.join(site_packages_dir, "REALM_LIBS.pth")
    with open(pth_file_path, "w") as out_file:
        out_file.write(project_path)
    print(f"Added project root to {pth_file_path}.")


def remove_venv(venv_name):
    if os.path.exists(venv_name):
        shutil.rmtree(venv_name)
        print(f"Removed '{venv_name}'.")
    else:
        print(f"'{venv_name}' not found, skipping.")


def run_runtime_ini_setup():
    try:
        import add_runtime_ini
        add_runtime_ini.main()
    except ImportError as e:
        print(f"ERROR: Failed to import add_runtime_ini.py: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: add_runtime_ini.py failed: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Install or uninstall REALM virtual environments.")
    parser.add_argument("--data_analysis", action="store_true",
                        help="Include the realm_analysis_venv for data analysis.")
    parser.add_argument("--uninstall", action="store_true",
                        help="Remove virtual environments instead of creating them.")
    args = parser.parse_args()

    project_path = os.getcwd()

    if args.uninstall:
        print("Uninstalling REALM virtual environments...")
        remove_venv("realm_core_venv")
        if args.data_analysis:
            remove_venv("realm_analysis_venv")
        print("\nUninstall complete.")
        return

    print(f"Project path: {project_path}")
    python_executable = find_python_version()

    # Always build core venv
    create_venv(python_executable, "realm_core_venv")
    install_requirements("realm_core_venv", os.path.join(SETUP_DIR, "requirements_webots.txt"))
    add_project_to_path("realm_core_venv", project_path)
    run_runtime_ini_setup()

    # Optionally build analysis venv
    if args.data_analysis:
        create_venv(python_executable, "realm_analysis_venv")
        install_requirements("realm_analysis_venv", os.path.join(SETUP_DIR, "requirements_analysis.txt"))
        add_project_to_path("realm_analysis_venv", project_path)

    print("\nSetup complete.")
    if args.data_analysis:
        print("  Environments created: realm_core_venv, realm_analysis_venv")
    else:
        print("  Environment created: realm_core_venv")
        print("  Run with --data_analysis to also create realm_analysis_venv.")


if __name__ == "__main__":
    main()
