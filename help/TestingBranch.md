# Update

Update subrepos to latest versions whenever you try to run the testing branch 

```
cd ~/MowgliRover
git submodule update --recursive
cd ~/MowgliRover/src/Mowgli-open_mower_ros/
git checkout main
git pull
cd ~/MowgliRover
depit
```

Then recompile with ./scripts/build_all.sh
