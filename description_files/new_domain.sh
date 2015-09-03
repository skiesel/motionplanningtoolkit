if [ $# -eq 0 ]
  then
    echo "No domain name supplied... aborting...";
    exit;
fi

DOMAIN=$1

mkdir -p $DOMAIN $DOMAIN/algorithms $DOMAIN/instances
touch $DOMAIN/$DOMAIN.dom

ALGORITHMS="rrt sst sstgrid rrtconnect kpiece pprm"

for ALG in $ALGORITHMS;
do
	touch $DOMAIN/algorithms/$ALG.alg
done