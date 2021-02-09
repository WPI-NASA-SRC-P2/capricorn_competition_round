import subprocess, os

def runTestsInPackage(pkg):
    # Change our directory to the provided package, so that we can run tests for a single package
    os.chdir(f"/home/srcp2/catkin_ws/src/capricorn_competition_round/{pkg}")

    # Run the command `catkin run_tests --nodeps --this`, which will run tests available in the CWD. Stores the result in `res`
    res = subprocess.run(['catkin', 'run_tests', '--no-deps', '--this'], stdout=subprocess.PIPE)

    # Turn bytes into utf-8, and split based on newline
    lines = res.stdout.decode("utf-8").split("\n")

    # These following lines are to grab just gtest output, as catkin clutters stdout with a lot of unnecessary logs.
    # Does this work in all cases? Almost certainly not. Does it work in the extremely limited cases I've tested it in? Absolutely!
    start_idx = 0
    end_idx = 0

    for i, line in enumerate(lines):
        if "run_tests.py" in line and start_idx == 0:
            start_idx = i + 3
            continue
        if "run_tests.py: verify result" in line and not i == start_idx:
            end_idx = i
            break

    # Grab the lines we are interested in
    test_lines = lines[start_idx:end_idx]

    return "\n".join(test_lines)

if __name__ == "__main__":
    print("Pretty test formatter. Please note that if your tests don't build properly, this is not guaranteed to run as expected.")
    print("In these cases, there will probably be no stdout. Please run `catkin run_tests` if this is the case.")
    print("If your tests require ROS (i.e. startup a node), please run `roscore` before this script.")
    package = input("Package to run tests for: ")

    print("Running tests, output will not show until all tests are complete...\n\n")

    print(runTestsInPackage(package))