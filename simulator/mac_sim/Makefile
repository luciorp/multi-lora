
CCFLAGS = -DSIMULATION

all:  firmware

firmware: ../../main/lora_mesh_main.c base.c base.h
	gcc $(CCFLAGS) -lpthread -o firmware ../../main/lora_mesh_main.c base.c

clean:
	rm -f firmware 
