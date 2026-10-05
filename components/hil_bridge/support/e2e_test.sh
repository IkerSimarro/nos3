#!/bin/bash
#
# End-to-end test of the HIL bridge without hardware or the full NOS3 launch.
# Starts a NOS Engine server, the NOS3 time driver, the real EPS and sample sims (with built-in data
# providers instead of 42), stand-in EGSE endpoints (COSMOS umbilical, ground link emulator, torquer
# sim), the bridge, and the fake MCU over a pty, then runs the fake MCU self-test.
#
# Run from the NOS3 root after `make config && make sim`:
#   docker run --rm -v $PWD:$PWD --add-host nos-engine-server:127.0.0.1 \
#       --add-host sc01-nos-engine-server:127.0.0.1 ivvitc/nos3-64:20260619 \
#       $PWD/components/hil_bridge/support/e2e_test.sh
#
HERE=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BASE_DIR=$( cd -- "$HERE/../../.." && pwd )
SIM_BIN=$BASE_DIR/sims/build/bin

BRIDGE=${BRIDGE:-$SIM_BIN/nos3-hil-bridge}
if [ ! -x "$BRIDGE" ]; then
    BRIDGE=$BASE_DIR/sims/build/hil_bridge/nos3-hil-bridge
fi
if [ ! -x "$BRIDGE" ]; then
    echo "nos3-hil-bridge not built; run make sim first"
    exit 1
fi

WORK=$(mktemp -d)
cleanup()
{
    kill $(jobs -p) 2> /dev/null
    sleep 1
    kill -9 $(jobs -p) 2> /dev/null
}
trap cleanup EXIT

cp $BASE_DIR/cfg/build/sims/sim_log_config.xml $WORK/
sed -e 's/GENERIC_EPS_42_PROVIDER/GENERIC_EPS_PROVIDER/' -e 's/SAMPLE_42_PROVIDER/SAMPLE_PROVIDER/' $BASE_DIR/cfg/build/sims/sc-1-nos3-simulator.xml > $WORK/sim.xml

# The server has an interactive menu and exits on stdin EOF, so keep stdin open
sleep infinity | nos_engine_server_standalone -f $SIM_BIN/nos_engine_server_config.json > $WORK/server.log 2>&1 &
sleep 1
(cd $WORK && $SIM_BIN/nos3-single-simulator -f $WORK/sim.xml generic-eps-sim > $WORK/eps.log 2>&1) &
(cd $WORK && $SIM_BIN/nos3-single-simulator -f $WORK/sim.xml sample-sim > $WORK/sample.log 2>&1) &

# The real NOS3 time driver, so TIME frames carry advancing simulation time
# It draws a curses screen, so it needs a terminal: `script` provides a pseudo-terminal
(cd $WORK && sleep infinity | TERM=xterm script -qfec "$SIM_BIN/nos3-single-simulator -f $WORK/sim.xml time" \
    /dev/null > $WORK/time.log 2>&1) &

# Stand-ins for the EGSE endpoints (COSMOS umbilical, ground link emulator, torquer sim):
# each answers the fake MCU's test message so both directions are checked
python3 - > $WORK/egse.log 2>&1 <<'EOF2' &
import selectors, socket
def bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port))
    return s
out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
umb, rf, trq = bind(9011), bind(9021), bind(14242)
sel = selectors.DefaultSelector()
for s in (umb, rf, trq):
    sel.register(s, selectors.EVENT_READ)
while True:
    for key, _ in sel.select():
        data, _ = key.fileobj.recvfrom(65536)
        print("got", key.fileobj.getsockname()[1], data, flush=True)
        if key.fileobj is umb and data == b"umb-tm-test":
            out.sendto(b"umb-tc-test", ("127.0.0.1", 9010))
        elif key.fileobj is rf and data == b"rf-tx-test":
            out.sendto(b"\xff\x88\x14\x00rf-rx-test", ("127.0.0.1", 9020))  # RSSI -120 dBm, SNR 5 dB
        elif key.fileobj is trq:
            out.sendto(b"trq:" + data, ("127.0.0.1", 9010))
EOF2

python3 $HERE/hil_fake_mcu.py --pty $WORK/tty --test --check-uart --check-egse --check-time --wait 30 &
FAKE=$!
sleep 1
$BRIDGE -d $WORK/tty --umb-host 127.0.0.1 --rf-host 127.0.0.1 --trq-host 127.0.0.1 -v > $WORK/bridge.log 2>&1 &

wait $FAKE
RC=$?

echo "---- bridge log ----"
cat $WORK/bridge.log
if [ $RC -ne 0 ]; then
    echo "---- eps sim log ----"
    tail -20 $WORK/eps.log
    echo "---- sample sim log ----"
    tail -20 $WORK/sample.log
    echo "---- time driver log ----"
    tail -20 $WORK/time.log
    echo "---- EGSE stand-in log ----"
    cat $WORK/egse.log
fi
echo
[ $RC -eq 0 ] && echo "E2E RESULT: PASS" || echo "E2E RESULT: FAIL"
exit $RC
