cd ..
cscript scripts\dos2unix.vbs %cd%\dockerfile
cd scripts
copy /y docker.sav docker.sh
cscript dos2unix.vbs "docker.sh"
cd ..
rem docker build . -t capstone
docker build --no-cache . -t capstone
docker run -p 4567:4567 -v %cd%:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
set /p=

