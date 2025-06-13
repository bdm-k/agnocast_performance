set -e
set -u
set -o pipefail

if [ ! -d pidstat_results ]
then
  echo "The pidstat_results directory was not found. Run this script from the project root."
  exit 1
fi

TALKER_LOG_FILE=$(date +%s%3N).log
sleep 1  # Make sure the file names do not conflict
LISTENER_LOG_FILE=$(date +%s%3N).log

echo "{"
echo "  \"talker_path\": \"$TALKER_LOG_FILE\","
echo "  \"listener_path\": \"$LISTENER_LOG_FILE\""
echo "}"

pidstat -p $(pgrep talker | paste -sd, -) 1 60 > pidstat_results/$TALKER_LOG_FILE &
pidstat -p $(pgrep listener | paste -sd, -) 1 60 > pidstat_results/$LISTENER_LOG_FILE &

echo "collecting stats..."

wait  # Wait for the background jobs to finish
echo "done"
