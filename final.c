#include <u.h>
#include <libc.h>

/******************* IMU function call *****************/
void checkDevice(int i2cfd, int addr)	{
	char reg = 0x00;
	if (write(i2cfd, &reg, 1) !=1)	{
		print("Cannot reset device at register 0x%02x, sensor addr 0x%02x \n", reg, addr);
		exits("failed to reset device");
	}
}
void bno_reset (int i2cfd)	{
	uchar data[2];
	data[0] = 0x3f;
	data[1] = 0x20;
	if (write(i2cfd, data, 1) !=1)	{
		print("Cannot access device at register 0x%02x \n", data[0]);
		exits("failed to access device");
	}
	sleep(10000);
}
void setOperationMode (int i2cfd, uchar mode)	{
	uchar buf[2];
	buf[0] = 0x3d;
	buf[1] = mode;
	if (pwrite(i2cfd, buf, 2,0x00) !=2)	{
		print("Cannot change mode device%r\n");
		exits("failed to config device");
	}
}
void get_eul(int i2cfd, double *eul)	{
	char reg = 0x1a;
	int buf;
	if (pwrite(i2cfd, &reg, 1,0x00) !=1)	{
		print("Error1: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	uchar data[6] = {0,0,0,0,0,0};
	if (pread(i2cfd,data, 6,0x00) !=6)	{
		print("Error2: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	// get head (x) value
	buf = ((int) data[1] <<8) | data[0];
	eul[0] = (double) buf/16.0;
	eul[0] = (eul[0] > 100.0) ? 0 : eul[0];

	// get roll (y) value
	buf = ((int) data[3] <<8) | data[2];
	eul[1] = (double) buf/16.0;
	eul[1] = (eul[1] > 100.0) ? 0 : eul[1];
	
	//get pitch (z) value
	buf = ((int) data[5] <<8) | data[4];
	eul[2] = (double) buf/16.0;
	eul[2] = (eul[2] > 100.0) ? 0 : eul[2];

}

// read gyroscope field
void get_gyr(int i2cfd, double *gyr)	{
	char reg = 0x14;
	int buf;
	if (pwrite(i2cfd, &reg, 1,0x00) !=1)	{
		print("Error1: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	uchar data[6] = {0,0,0,0,0,0};
	if (pread(i2cfd,data, 6,0x00) !=6)	{
		print("Error2: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	// get head (x) value
	buf = ((int) data[1] <<8) | data[0];
	gyr[0] = (double) buf/16.0;
	gyr[0] = (gyr[0] >  200.0) ? 0.0 : gyr[0];

	// get roll (y) value
	buf = ((int) data[3] <<8) | data[2];
	gyr[1] = (double) buf/16.0;
	gyr[1] = (gyr[1] > 200.0) ? 0 : gyr[1];
	
	//get pitch (z) value
	buf = ((int) data[5] <<8) | data[4];
	gyr[2] = (double) buf/16.0;
	gyr[2] = (gyr[2] > 200.0) ? 0 : gyr[2];

}
// Read Quaternation data
void 
get_quat (int i2cfd, double *quat)	{
	char reg = 0x20;
	int buf;
	if (write(i2cfd, &reg, 1) !=1)	{
		print("Error1: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	uchar data[8] = {0,0,0,0,0,0,0,0};
	if (read(i2cfd,data, 8) !=8)	{
		print("Error2: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	// get (w) value
	buf = ((int) data[1] <<  8) | data[0];
	quat[0] = (double) buf/ (1 << 14);

	// get (x) value
	buf = ((int) data[3] <<8) | data[2];
	quat[1] = (double) buf/16.0;

	// get (y) value
	buf = ((int) data[5] <<8) | data[4];
	quat[2] = (double) buf/ (1 << 14);
	
	//get (z) value
	buf = ((int) data[7] <<8) | data[6];
	quat[3] = (double) buf/ (1 << 14);
}

void 
calculatePosition( double *position, double* acc, double dt)	{
	int i;
	for (i =0; i < 3; i++)	{
		position[i] += acc[i]*dt*dt;
	}
}

void
printOrientValue(double head, double roll, double pitch)	{
	print("X:\t %.5f, Y:\t %.5f, Z:\t %.5f\n", head, roll, pitch);
}

// get linear acceleration
void
get_linear (int i2cfd, double *acc)	{
	char reg = 0x28;
	char unit_cmd = 0x3b;
	int buf =0;
	double ufact = 1.0;
	//check unit
	if (pwrite(i2cfd, &unit_cmd, 1,0x00) !=1)	{
		print("Error1: Cannot read data device at register 0x%02x \n", unit_cmd);
		exits("failed to read reg");
	}
	uchar unit_sel;
	if (pread(i2cfd,&unit_sel, 1,0x00) !=1)	{
		print("Error2: Cannot read data device at register 0x%02x \n", unit_cmd);
		exits("failed to read reg");
	}
	if ((unit_sel >>0) & 0x01)	{
		 ufact =1.0;
	}	else	{
		ufact=100.0;		}
	
	// get linear acceleration data
	if (pwrite(i2cfd, &reg, 1,0x00) !=1)	{
		print("Error1: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	uchar data[6] = {0,0,0,0,0,0};
	if (pread(i2cfd,data, 6,0x00) !=6)	{
		print("Error2: Cannot read data device at register 0x%02x \n", reg);
		exits("failed to read reg");
	}
	// get head (x) value
	buf = ((int) data[1] <<8) | data[0];
	acc[0] = (double) buf/ ufact;
	acc[0] = (acc[0] > 300.0) ? 0 : acc[0];

	// get roll (y) value
	buf = ( (int) data[3] <<8) | data[2];
	acc[1] = (double) buf/ ufact;
	acc[1] = (acc[1] > 300.0) ? 0 : acc[1];

	//get pitch (z) value
	buf = ( (int) data[5] <<8) | data[4];
	acc[2] = (double) buf/ ufact;
	acc[2] = (acc[2] > 300.0) ? 0 : acc[2];

}

enum {
	MAGIC = 740,
};

void
usleep(ulong usec)		{
	int i,j;

	for (i =0; i <usec; i++)	{
		for (j  =0; j <MAGIC; j++);
	}
}
/***************************** END IMU Function Call*********************/

/****************************** Display function Call *********************/
#define Height		0x83
#define Width		0xa1
long safepwrite(int spifd, void* buff, long bytes, vlong addr) {
    long output = pwrite(spifd, buff, bytes, addr);
    if(output < 0) {
        print("Failed pwrite\n");
        exits("failed pwrite");
    }
    return output;
}

void spicmd(uchar c, int commandfd, int spifd)
{
    fprint(commandfd, "set 7 0");
    safepwrite(spifd, &c, 1, 0x00);
}
void spidata(uchar *p, int n, int commandfd, int spifd)
{
    fprint(commandfd, "set 7 1");
    safepwrite(spifd, p, n, 0x00);
    fprint(commandfd, "set 7 0");

}
void
setwindow(int gpio, int spi, int x0, int y0, int x1, int y1)	{
	uchar buff[4];
    // set column address
    spicmd(0x2a, gpio, spi);
    buff[0] = 0;
    buff[1] = x0;
    buff[2] = 0;
    buff[3] = x1;
    spidata(buff, 4, gpio, spi);

    // set row address
    spicmd(0x2b, gpio, spi);
    buff[0] =0;
    buff[1] = y0;
    buff[2] =0;
    buff[3] = y1;
    spidata(buff, 4, gpio, spi);
}
void
drawPixel(int gpio, int spi, int x0, int y0)	{
	uchar buff[3];
	if ( (x0 >= Width) || (y0 >= Height))
		return;
	setwindow(gpio, spi, x0, y0, x0 +1, y0+ 1);
	spicmd(0x2c, gpio,spi);
	buff[0] = 0xff;
	buff[1] = 0xff;
	buff[2] = 0xff;
	spidata(buff,3, gpio, spi);
}
void
drawCircle(int gpio, int spi, int x0, int y0, int r)	{
	int f, ddF_x, ddF_y, x, y;
	f = 1-r;
	ddF_x =1;
	ddF_y =-2*r;
	x =0, y =r;
	drawPixel(gpio, spi, x0, y0+r);
	drawPixel(gpio, spi, x0, y0-r);
	drawPixel(gpio, spi, x0+r, y0);
	drawPixel(gpio, spi, x0-r, y0);
	while(x<y)	{
		if (f >=0)	{
			y--;
			ddF_y +=2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		drawPixel(gpio, spi, x0+x, y0+y);
		drawPixel(gpio, spi, x0-x, y0+y);
		drawPixel(gpio, spi, x0+x, y0-y);
		drawPixel(gpio, spi, x0-x, y0-y);
		drawPixel(gpio, spi, x0+y, y0+x);
		drawPixel(gpio, spi, x0-y, y0+x);
		drawPixel(gpio, spi, x0+y, y0-x);
		drawPixel(gpio, spi, x0-y, y0-x);
	}
}
void
clearDisplay(int gpio, int spi)	{
	uchar buff[3];
	int i,j,b;
	setwindow(gpio, spi, 0, 0, Width, Height);
    spicmd(0x2c, gpio, spi);
    // This makes an interesting diagonal shape with gaps in between
    for(i=0; i < Width; i++) {
        for(j = 0; j < Height; j++) {
		buff[0] = 0x00;
		buff[1] = 0x00;
		buff[0] = 0x00;
		spidata(buff, 3, gpio, spi);
        }
    }
}
/**************************** END Display Function Call *****************/
//function to map value from one to another
double
mapValue( double value, double fromLow, double fromHigh, double toLow, double toHigh)	{
	return toLow + (value -fromLow) * (toHigh -toLow) / (fromHigh - fromLow);
}

//function to map x and y value
void mapXY( double x_in, double y_in, int *xy_out)	{
	x_in = (x_in >=30 ) ? 30.0 : x_in;
	y_in = (y_in >=30 ) ? 30.0 : y_in;
	xy_out[0] = (int) mapValue(x_in, 0.0, 30, 16.0, 154.0);
	xy_out[1] = (int) mapValue(y_in, 0.0, 30, 16.0, 115.0);
}
void 
main()
{
 /*************** Initialize i2c imu ******************/
	int fd;
	int i2c_addr = 28;
	double gyr[3] = {999.9, 999.9, 999.9};
	// The default address of BNO055 is (0x29), and alternative address is (0x28)
	fd = open("/dev/i2c.28.data", ORDWR);
	if (fd <0) {
		bind("#J28","/dev",MAFTER);
		fd = open ("/dev/i2c.28.data", ORDWR);
		if (fd <0 )	{
			fprint(2,"Open failed: %r\n");
			exits("No device");
		}
	}
	checkDevice (fd, i2c_addr);
	uchar mode = 0x08;
	setOperationMode(fd, mode);
	double dt = 0.01;
	/************ end initialize *************/
	
	/************** initialize spi display **********/
    uchar buff[128];
    int i;
    int j;
    int b;
    int gpio; // chip select file description this is on gpio pin 8
    int spifd;
    int numcols = 0x83;
    int numrows = 0xa1;
    gpio = open("/dev/gpio", ORDWR);
    if(gpio < 0) {
        bind("#G", "/dev", MAFTER);
        gpio = open("/dev/gpio", ORDWR);
        if(gpio < 0) {
            print("Open failed: %r\n");
            exits(nil);
        }
    }

    fprint(gpio, "function 7 out");
    fprint(gpio, "float 7");
    fprint(gpio, "set 7 0");
    spifd = open("spi0", ORDWR);
    if (spifd < 0) {
        bind("#π", ".", MAFTER);
        spifd = open("spi0", ORDWR);
        if (spifd < 0) {
            print("Open failed: %r\n");
            exits(nil);
        }
    }
	/********** display set up ****************/
    // software reset
    spicmd(0x01, gpio, spifd);
    sleep(500);
    spicmd(0x11, gpio, spifd);
    sleep(200);
    spicmd(0x29, gpio, spifd);on
    spicmd(0x13, gpio, spifd);
    spicmd(0x36, gpio, spifd);
    buff[0] = 0x00;
    spidata(buff, 1, gpio, spifd);
    spicmd(0x38, gpio, spifd);
    sleep(2000);


	sleep(2000);
	print("start to clear display\n");
	clearDisplay(gpio, spifd);
	sleep(2000);
	print("draw a circle\n");
	drawCircle(gpio, spifd, 11, 11, 10);
	sleep(2000);
	/***************** End initialize display **************/
	
	int xy_out[2] = {16, 16};
	int r = 10;
	while (1)	{
		clearDisplay(gpio, spifd);
		sleep(50);
		// print a first circle
		drawCircle(gpio, spifd, xy_out[0], xy_out[1], r);
		sleep(500);
		// get value from gyroscope
		get_gyr(fd, gyr);
		printOrientValue(gyr[0], gyr[1], gyr[2]);
		mapXY(gyr[0], gyr[1], xy_out);
		usleep(500000);
	}
}	
