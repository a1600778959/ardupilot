#!/usr/bin/env bash
# Rover/dimah743 CI helper.

XOLDPWD=$PWD  # profile changes directory :-(

. ~/.profile

cd $XOLDPWD

set -ex

c_compiler=${CC:-gcc}
cxx_compiler=${CXX:-g++}

export BUILDROOT=/tmp/ci.build
rm -rf $BUILDROOT
export GIT_VERSION="abcdef"
export GIT_VERSION_INT="15"
export CHIBIOS_GIT_VERSION="12345667"
export CCACHE_SLOPPINESS="include_file_ctime,include_file_mtime"

if [ -z "$CI_BUILD_TARGET" ]; then
    CI_BUILD_TARGET="sitltest-rover dimah743"
fi

waf=modules/waf/waf-light

echo "Targets: $CI_BUILD_TARGET"
echo "Compiler: $c_compiler"

pymavlink_installed=0
mavproxy_installed=0

function install_pymavlink() {
    if [ $pymavlink_installed -eq 0 ]; then
        echo "Installing pymavlink"
        git submodule update --init --recursive --depth 1
        (cd modules/mavlink/pymavlink && python3 -m pip install --user .)
        pymavlink_installed=1
    fi
}

function install_mavproxy() {
    if [ $mavproxy_installed -eq 0 ]; then
        echo "Installing MAVProxy"
        pushd /tmp
          git clone https://github.com/ardupilot/MAVProxy --depth 1
          pushd MAVProxy
            python3 -m pip install --user --force .
          popd
        popd
        mavproxy_installed=1
        python3 -m pip uninstall -y pymavlink
    fi
}

function run_autotest() {
    NAME="$1"
    BVEHICLE="$2"
    RVEHICLE="$3"

    cat /proc/cpuinfo

    install_mavproxy
    install_pymavlink
    unset BUILDROOT
    echo "Running SITL $NAME test"

    w=""
    if [ "$c_compiler" == "clang" ]; then
        w="$w --check-c-compiler=clang --check-cxx-compiler=clang++"
    fi
    if [ "$NAME" == "Rover" ]; then
        w="$w --enable-math-check-indexes"
    fi
    if [ "x$CI_BUILD_DEBUG" != "x" ]; then
        w="$w --debug"
    fi
    if [ "$NAME" == "Examples" ]; then
        w="$w --speedup=5 --timeout=14400 --debug --no-clean"
    fi
    Tools/autotest/autotest.py --show-test-timings --junit --waf-configure-args="$w" "$BVEHICLE" "$RVEHICLE"
    ccache -s && ccache -z
}

for t in $CI_BUILD_TARGET; do
    if [ "$t" == "sitltest-rover" ]; then
        sudo apt-get update || /bin/true
        sudo apt-get install -y ppp || /bin/true
        run_autotest "Rover" "build.Rover" "test.Rover"
        continue
    fi

    if [ "$t" == "unit-tests" ]; then
        run_autotest "Unit Tests" "build.unit_tests" "run.unit_tests"
        continue
    fi

    if [ "$t" == "examples" ]; then
        ./waf configure --board=sitl --debug
        ./waf examples
        run_autotest "Examples" "--no-clean" "run.examples"
        continue
    fi

    if [ "$t" == "replay" ]; then
        echo "Building replay"
        $waf configure --board sitl --debug --disable-scripting
        $waf replay
        echo "Building AP_DAL standalone test"
        $waf configure --board sitl --debug --disable-scripting --no-gcs
        $waf --target tool/AP_DAL_Standalone
        $waf clean
        continue
    fi

    if [ "$t" == "validate_board_list" ]; then
        echo "Validating board list"
        ./Tools/autotest/validate_board_list.py
        continue
    fi

    if [ "$t" == "check_autotest_options" ]; then
        echo "Checking autotest options"
        install_mavproxy
        install_pymavlink
        ./Tools/autotest/autotest.py --help
        ./Tools/autotest/autotest.py --list
        ./Tools/autotest/autotest.py --list-subtests
        continue
    fi

    if [ "$t" == "python-cleanliness" ]; then
        echo "Checking Python code cleanliness"
        ./Tools/scripts/run_flake8.py
        continue
    fi

    if [ "$t" == "astyle-cleanliness" ]; then
        echo "Checking AStyle code cleanliness"
        ./Tools/scripts/run_astyle.py --dry-run
        continue
    fi

    if [ "$t" == "configure-all" ]; then
        echo "Checking configure of all boards"
        ./Tools/scripts/configure_all.py --build-target rover
        continue
    fi

    if [ "$t" == "build-options-defaults-test" ]; then
        install_pymavlink
        echo "Checking default options in build_options.py work"
        time ./Tools/autotest/test_build_options.py \
             --no-disable-all \
             --no-disable-none \
             --no-disable-in-turn \
             --no-enable-in-turn \
             --board=dimah743 \
             --build-targets=rover
        echo "Checking all/none options in build_options.py work"
        time ./Tools/autotest/test_build_options.py \
             --no-disable-in-turn \
             --no-enable-in-turn \
             --build-targets=rover
        continue
    fi

    if [ "$t" == "param_parse" ]; then
        python3 Tools/autotest/param_metadata/param_parse.py --vehicle Rover
        continue
    fi

    if [ "$t" == "logger_metadata" ]; then
        python3 Tools/autotest/logger_metadata/parse.py --vehicle Rover
        continue
    fi

    if [[ -z ${CI_CRON_JOB+1} ]]; then
        echo "Starting waf Rover build for board ${t}..."
        $waf configure --board "$t" \
                --enable-benchmarks \
                --enable-header-checks \
                --check-c-compiler="$c_compiler" \
                --check-cxx-compiler="$cxx_compiler"
        $waf clean
        $waf rover
        ccache -s && ccache -z
        continue
    fi
done

echo build OK
exit 0
