DEVICE=$1

case $DEVICE in

  porygon)
    echo -n "~/.ssh/tj2_porygon"
    ;;

  porygon2)
    echo -n "~/.ssh/tj2_porygon2"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
