usage() {
cat << EOF
************************************************************************************************************************************
ATTACH SCRIPT FOR PRRI SIMULATION EXPERIMENT WITH TWO ICUBS COMANIPULATING A BOARD
Author:  Yeshasvi Tirupachuri   <yeshasvi.tirupachuri@iit.it>
This script takes name of the human model and attaches the hands of the human to iCub
USAGE:
        $0 options
************************************************************************************************************************************
OPTIONS: <command specifier>
************************************************************************************************************************************
EXAMPLE USAGE: ./handlesHandsLinkAttacher.sh attach -----> Attach the handle links of the board to the hand links of the two robot
EXAMPLE USAGE: ./handlesHandsLinkAttacher.sh detach -----> Detach the handle links of the board from the hand links of the two robot
************************************************************************************************************************************
EOF
}

attach() {
  echo "attachUnscoped board side1_left_dummy_link iCub1 l_hand" | yarp rpc /board/linkattacher/rpc:i
  echo "attachUnscoped board side1_right_dummy_link iCub1 r_hand" | yarp rpc /board/linkattacher/rpc:i
  echo "attachUnscoped board side2_left_dummy_link iCub2 r_hand" | yarp rpc /board/linkattacher/rpc:i
  echo "attachUnscoped board side2_right_dummy_link iCub2 l_hand" | yarp rpc /board/linkattacher/rpc:i
}

detach() {
  echo "detachUnscoped board side1_left_dummy_link" | yarp rpc /board/linkattacher/rpc:i
  echo "detachUnscoped board side1_right_dummy_link" | yarp rpc /board/linkattacher/rpc:i
  echo "detachUnscoped board side2_left_dummy_link" | yarp rpc /board/linkattacher/rpc:i
  echo "detachUnscoped board side2_right_dummy_link" | yarp rpc /board/linkattacher/rpc:i
}

################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
if [[ $# -eq 0 || $# -lt 1 ]] ; then
    echo "Error in options passed!"
    echo ""
    usage
    exit 1
fi

COMMAND=$1

if [[ ${COMMAND} == "attach" ]] ; then
  attach
fi

if [[ ${COMMAND} == "detach" ]] ; then
  detach
fi

exit 0
