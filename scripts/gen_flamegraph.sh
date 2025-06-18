set -e
set -u
set -o pipefail

if [ $# -ne 1 ]
then
  echo "Usage: bash $0 <record_data>"
  exit 1
fi

if [ $(stat -c '%U' $1) != $(whoami) ]
then
  sudo chown $(whoami):$(whoami) $1
  chmod 666 $1
fi

workspace=$(mktemp -d)
base=$(basename $1)

perf script -i $1 > $workspace/a.perf
stackcollapse-perf.pl $workspace/a.perf > $workspace/a.folded
flamegraph.pl $workspace/a.folded > perf_results/flamegraph/${base%.*}.folded.svg

rm -r $workspace
