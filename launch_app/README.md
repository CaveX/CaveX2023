# launch_app
Services and shell scripts to make CaveX work when the primary controller boots

The services in launch_services must be placed inside the /lib/systemd? folder on your Ubunutu 18.04 device. Each service runs a corresponding shell script.

The shell scripts in launch_scripts launch ros nodes, run python scripts, and more. 

You must enable/disable, start/stop services using systemd to configure the startup app. Matt based it off this blog https://alesnosek.com/blog/2016/12/04/controlling-a-multi-service-application-with-systemd/.

At the end of 2021, the start up worked like this:

![app drawio](https://user-images.githubusercontent.com/92500933/140847594-edf788e4-da99-4eee-9748-efc180bb986d.png)
