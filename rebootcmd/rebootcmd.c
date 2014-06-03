#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <cutils/properties.h>
#include <android/log.h>

#define ALOGD(...) __android_log_print(ANDROID_LOG_DEBUG, "rebootcmd: ", __VA_ARGS__)

int main(int argc, char**argv, char *envp[])
{
	if(strcmp(argv[1], "recovery") == 0){
		ALOGD("Rebooting into recovery");
		system("/system/bin/moboot_control.sh recovery");
	} else {
		if(strcmp(argv[1], "altos") == 0){
			ALOGD("Rebooting into WebOS");
			system("/system/bin/moboot_control.sh altos");
		}
	}
	return -1;
}
