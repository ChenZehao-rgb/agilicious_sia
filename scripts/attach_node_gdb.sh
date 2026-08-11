MYPID='';
while [ -z "$MYPID" ]
do
	MYPID=$(rosnode info $1 | grep -oP '(?<=Pid: )[0-9]+')
done
sudo gdb $2 $MYPID -ex cont
