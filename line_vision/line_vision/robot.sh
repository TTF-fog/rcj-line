#!/bin/bash
# ./robot.sh start [args] | stop | restart | status | log | enable | disable
# enable adds a crontab @reboot line that runs "start". Default args: --view
cd "$(dirname "$(readlink -f "$0")")" || exit 1
PID=runs/robot.pid
OUT=runs/logs/robot.out
CRON="@reboot sleep 10 && $PWD/robot.sh start"

running() { [ -f $PID ] && kill -0 "$(cat $PID)" 2>/dev/null; }

case "$1" in
  start)
    running && { echo "already running (pid $(cat $PID))"; exit 0; }
    shift
    args="${*:---view}"
    mkdir -p runs/logs
    # Restarts the robot if it crashes. Waiting for the board first stops a missing board from making a log file
    # every restart.
    setsid bash -c "
      while true; do
        [[ '$args' == *--dry* ]] || until ls /dev/ttyACM* > /dev/null 2>&1; do sleep 2; done
        .venv-yolo/bin/python -m robot $args
        echo \"exited \$?, restarting in 5 s\"
        sleep 5
      done" >> $OUT 2>&1 < /dev/null &
    echo $! > $PID
    echo "started (pid $!), log: $OUT"
    ;;
  stop)
    running || { echo "not running"; rm -f $PID; exit 0; }
    kill -- -"$(cat $PID)"
    rm -f $PID
    echo "stopped"
    ;;
  restart)
    "$0" stop
    sleep 2
    shift
    "$0" start "$@"
    ;;
  status)
    running && echo "running (pid $(cat $PID))" || echo "not running"
    crontab -l 2>/dev/null | grep -qxF "$CRON" && echo "starts at boot" || echo "does not start at boot"
    ;;
  log)
    tail -n 50 -f $OUT
    ;;
  enable)
    (crontab -l 2>/dev/null | grep -vxF "$CRON"; echo "$CRON") | crontab -
    echo "added to crontab: $CRON"
    ;;
  disable)
    crontab -l 2>/dev/null | grep -vxF "$CRON" | crontab -
    echo "removed from crontab"
    ;;
  *)
    echo "usage: $0 start [args] | stop | restart | status | log | enable | disable"
    exit 1
    ;;
esac
