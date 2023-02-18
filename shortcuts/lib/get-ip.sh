DEVICE=$1

case $DEVICE in

  porygon)
    echo -n "10.0.88.44"
    ;;

  porygon2)
    echo -n "10.0.88.35"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
