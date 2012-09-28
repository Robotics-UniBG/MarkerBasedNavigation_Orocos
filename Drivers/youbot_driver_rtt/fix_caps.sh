
OCL_BIN="`rospack find ocl`/bin/"
OTR_BIN="`rosstack find orocos_toolchain`/install/bin"

function do_setcap() {
    if [[ ! -d $1 ]]; then
	echo "fixcap error: no directory $1"
	exit -1
    fi
    pushd $1 &> /dev/null
    for i in deployer* rttlua*; do
	if [[ ! -x $i ]]; then continue; fi
	echo "configuring capabilities on $i";
	sudo setcap cap_sys_nice,cap_net_raw+ep $i
	#sudo setcap cap_net_raw+ep $i 
    done
    popd &> /dev/null
}

do_setcap $OCL_BIN
do_setcap $OTR_BIN

