#include <MLX90621.h>

#include <time.h>
#include <stdio.h>
#include <stdlib.h>

int main() {

    MLX90621 mlx_sensor;
    if(!mlx_sensor.Init()) {
        printf(" [ERROR:main] MLX90621 init failed! \n");
        return 0;
    }
    PINFO("\n [INFO:main] OK, MLX90621 init.");

    // Grab frames
    long int start_time;
    long int time_diff;
    struct timespec timer;
    short frame_count = 0;
    do {

        //clock_gettime(CLOCK_REALTIME, &timer);
        //start_time = timer.tv_nsec;

        float To = mlx_sensor.GetTo();
	++frame_count;
        if (frame_count % 30 == 0) {
        //    frame_count = 0;
            printf("AVG_To %.3f \n", To);
        }

        //clock_gettime(CLOCK_REALTIME, &timer);
        //time_diff = timer.tv_nsec - start_time;
        //if (time_diff < 0) time_diff += 1000000000;
        //printf(" Run time %ld \n", time_diff/1000000);

    } while (frame_count < 300);

    // release resources
    mlx_sensor.Deinit();
}
