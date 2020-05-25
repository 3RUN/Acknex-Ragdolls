
#include <acknex.h>
#include <default.c>

#define PRAGMA_POINTER

#define PRAGMA_PATH "resources"
#define PRAGMA_PATH "rg_parts"

#define OBJ_HEALTH skill50
#define OBJ_HAS_RAGDOLL skill51
#define OBJ_RAGDOLL_STRUCT skill52
#define OBJ_TYPE skill52
#define TYPE_NONE 0
#define TYPE_RD_BODY_PART 1

STRING *human_mdl = "human.mdl";

// <ackphysx.h> will bring trash into WED action list
#include "ackphysx.h"

#include "ragdoll.h"

var rigid_gravity = 9.81;

void npc_event()
{
	if(event_type == EVENT_CLICK)
	{
		my->emask &= ~ENABLE_CLICK;
		my->OBJ_HEALTH = -1;
		ragdoll_create(my);
	}
	
	if(event_type == EVENT_FRAME)
	{
		my->pan += 10 * (key_z - key_x) * time_step;
		
		if(my->OBJ_HAS_RAGDOLL == true)
		{
			ragdoll_update(my);
			
			my->skill4 -= time_frame / 16;
			if(my->skill4 <= 0)
			{
				ptr_remove(my);
			}
		}
		else if(my->OBJ_HEALTH > 0)
		{
			switch(my->skill2)
			{
				case 0:
				{
					my->skill1 += 4 * time_step;
					my->skill1 %= 100;
					ent_animate(my, "idle", my->skill1, ANM_CYCLE);
					break;
				}
				
				case 1:
				{
					my->skill1 += 8 * time_step;
					my->skill1 %= 100;
					ent_animate(my, "walk", my->skill1, ANM_CYCLE);
					break;
				}
				
				case 2:
				{
					my->skill1 += 8 * time_step;
					my->skill1 %= 100;
					ent_animate(my, "run", my->skill1, ANM_CYCLE);
					break;
				}
				
				case 3:
				{
					my->skill1 += 3 * time_step;
					my->skill1 %= 100;
					ent_animate(my, "jump", my->skill1, ANM_CYCLE);
					break;
				}
			}
		}
	}
}

void npc()
{
	set(my, NOFILTER | PASSABLE);	
	my->emask |= ENABLE_CLICK | ENABLE_FRAME;
	my->event = npc_event;
	
	my->OBJ_HEALTH = 100;
	my->skill2 = integer(random(4)); // random animation
	my->skill3 = integer(random(2)); // random skin
	my->skill4 = 30; // ragdoll lives for 30 seconds
	
	ent_cloneskin(my);
	my->skin = my->skill3 + 1;
}

void on_ent_remove_event(ENTITY *ent)
{
	physX_ent_remove(ent);
	
	if(ent->OBJ_HAS_RAGDOLL == true)
	{
		ragdoll_remove(ent);
	}
}

void on_frame_event()
{
	// call physX_update before entity actions
	physX_update();
	
	// slow motion effect
	time_factor = 1;
	if(key_space){ time_factor = 0.25; }
}

void main()
{
	on_ent_remove = on_ent_remove_event;
	on_frame = on_frame_event;
	
	camera->clip_far = 1024;
	vec_set(&d3d_lodfactor, vector(40, 50, 60));
	mip_levels = 8;
	
	video_mode = 9;
	fps_max = 60;
	fps_min = 30;
	time_smooth = 0.9;
	warn_level = 6;
	
	physX_open();
	pX_setunit(0.1);
	ph_fps_max_lock = 60;
	ph_check_distance = 2;
	pX_setccd(1);
	pX_setgravity(vector(0, 0, -rigid_gravity));
	
	level_load("");
	random_seed(0);
	
	vec_set(&camera->x, vector(211, -132, 189));
	vec_set(&camera->pan, vector(179, -26, 0));
	
	mouse_mode = 4;
	mouse_pointer = 2;
	
	ENTITY* ground_ent = ent_createterrain(NULL, nullvector, 100, 100, 1000);
	ent_setskin(ground_ent, bmap_fill(bmap_createblack(32, 32, 16), COLOR_BLUE, 100), 1);
	pXent_settype(ground_ent, PH_STATIC, PH_PLANE); // create a static plane at groundlevel zero
	pXent_setfriction(ground_ent, 10);
	
	// create npcs
	int i = 0, j = 0;
	
	for(i = 0; i < 5; i++)
	{
		for(j = 0; j < 5; j++)
		{
			ent_create(human_mdl, vector(0 - 64 * i, 0 - 64 * j, 32), npc);
		}
	}
	
	VECTOR force, speed, dist;
	ANGLE aforce, aspeed;
	
	vec_fill(&force, 0);
	vec_fill(&speed, 0);
	vec_fill(&dist, 0);
	
	vec_fill(&aforce, 0);
	vec_fill(&aspeed, 0);
	
	while(!key_esc)
	{
		time_factor = 1;
		if(key_space){ time_factor = 0.25; }
		
		sun_angle.pan += 5 * (key_cul - key_cur) * time_step; 
		sun_angle.tilt += 5 * (key_cuu - key_cud) * time_step; 
		
		aforce.pan = -5 * (mouse_right * mouse_force.x);
		aforce.tilt = 5 * (mouse_right * mouse_force.y);
		aforce.roll = 0;
		vec_add(&camera->pan, vec_accelerate(&dist, &aspeed, &aforce, 0.8));
		
		force.x = 7 * (key_w - key_s);
		force.y = 3 * (key_a - key_d);
		force.z = 3 * (key_q - key_e);
		vec_accelerate(&dist, &speed, &force, 0.5);
		vec_add(&camera->x, vec_rotate(&dist, &camera->pan));
		
		if(key_f)
		{
			pX_pick();
		}
		
		wait(1);
	}
}