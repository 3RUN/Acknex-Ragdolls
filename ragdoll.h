#ifndef _RAGDOLL_H_
	#define _RAGDOLL_H_

	// debug ragdoll body parts
	// this will disable physics for all body parts
	// and will help to see how each body part is placed
	// all adjustments of body parts has to be done in MED
	//#define DEBUG_RD_BODY_PARTS

	// total amount of body parts per each ragdoll
	#define MAX_RAGDOLL_PARTS 15

	// body part id
	#define RD_PELVIS 0
	#define RD_TORSO 1
	#define RD_HEAD 2
	#define RD_ARM_L_UP 3
	#define RD_ARM_L_DOWN 4
	#define RD_ARM_L_PALM 5
	#define RD_ARM_R_UP 6
	#define RD_ARM_R_DOWN 7
	#define RD_ARM_R_PALM 8
	#define RD_LEG_L_UP 9
	#define RD_LEG_L_DOWN 10
	#define RD_LEG_L_HEEL 11
	#define RD_LEG_R_UP 12
	#define RD_LEG_R_DOWN 13
	#define RD_LEG_R_HEEL 14

	// solver iteration count for body parts
	#define RAGDOLL_ITERATION 8 // default engine value is 4

	// other body part settings
	#define RAGDOLL_FRICTION 25
	#define RAGDOLL_RESTITUTION 50 // bounciness
	#define RAGDOLL_LINEAR_DAMPING 50
	#define RAGDOLL_ANGULAR_DAMPING 50
	#define RAGDOLL_BONE_RESTITUTION 0.025 // when out of limit bounciness (hinge joints)

	// 6d joint parameters
	#define RAGDOLL_6D_SPRING 0
	#define RAGDOLL_6D_DAMPING 100
	#define RAGDOLL_6D_LINEAR_LIMIT 0

	// ragdoll starting collision group
	// it increases for each body part with it's id
	// f.e pelvis - 3, torso - 4, head - 5, etc
	#define GROUP_RAGDOLL 3

	// ragdool parts (models)
	STRING *rg_pelvis_mdl = "rg_pelvis.mdl";
	STRING *rg_torso_mdl = "rg_torso.mdl";
	STRING *rg_head_mdl = "rg_head.mdl";
	STRING *rg_arm_l_up_mdl = "rg_arm_l_up.mdl";
	STRING *rg_arm_l_down_mdl = "rg_arm_l_down.mdl";
	STRING *rg_arm_l_palm_mdl = "rg_arm_l_palm.mdl";
	STRING *rg_arm_r_up_mdl = "rg_arm_r_up.mdl";
	STRING *rg_arm_r_down_mdl = "rg_arm_r_down.mdl";
	STRING *rg_arm_r_palm_mdl = "rg_arm_r_palm.mdl";
	STRING *rg_leg_l_up_mdl = "rg_leg_l_up.mdl";
	STRING *rg_leg_l_down_mdl = "rg_leg_l_down.mdl";
	STRING *rg_leg_l_heel_mdl = "rg_leg_l_heel.mdl";
	STRING *rg_leg_r_up_mdl = "rg_leg_r_up.mdl";
	STRING *rg_leg_r_down_mdl = "rg_leg_r_down.mdl";
	STRING *rg_leg_r_heel_mdl = "rg_leg_r_heel.mdl";

	// all bone names
	// structure bone names are set in ragdoll_init
	STRING *bone_pelvis_str = "Root_Hips";
	STRING *bone_torso_str = "Root_Chest";
	STRING *bone_head_str = "Root_Head";
	STRING *bone_arm_l_up_str = "Root_LeftArm";
	STRING *bone_arm_l_down_str = "Root_LeftForeArm";
	STRING *bone_arm_l_palm_str = "Root_LeftHand";
	STRING *bone_arm_r_up_str = "Root_RightArm";
	STRING *bone_arm_r_down_str = "Root_RightForeArm";
	STRING *bone_arm_r_palm_str = "Root_RightHand";
	STRING *bone_leg_l_up_str = "Root_LeftUpLeg";
	STRING *bone_leg_l_down_str = "Root_LeftLeg";
	STRING *bone_leg_l_heel_str = "Root_LeftFoot";
	STRING *bone_leg_r_up_str = "Root_RightUpLeg";
	STRING *bone_leg_r_down_str = "Root_RightLeg";
	STRING *bone_leg_r_heel_str = "Root_RightFoot";

	// d6 joint limits
	VECTOR *swing_head_ang = { x = 45; y = 0; z = 0; }
	VECTOR *twist_head_ang = { x = -45; y = 45; z = 0; }
	VECTOR *swing_arm_ang = { x = 90; y = 90; z = 0; }
	VECTOR *twist_arm_ang = { x = 0; y = 0; z = 0; }

	// 6d joint (used for upper arms and head)
	var joint_motion[6] =
	{
		NX_D6JOINT_MOTION_LOCKED,
		NX_D6JOINT_MOTION_LOCKED,
		NX_D6JOINT_MOTION_LOCKED,
		NX_D6JOINT_MOTION_LIMITED,
		NX_D6JOINT_MOTION_LIMITED,
		NX_D6JOINT_MOTION_LIMITED
	};

	typedef struct RAGDOLL
	{
		ENTITY *ghost_ent;
		ENTITY *rg_part[MAX_RAGDOLL_PARTS];
		STRING *bone[MAX_RAGDOLL_PARTS];
		VECTOR joint_limit[MAX_RAGDOLL_PARTS];
		VECTOR joint_axis[MAX_RAGDOLL_PARTS];
		var mass[MAX_RAGDOLL_PARTS];
		var joint_vertex[MAX_RAGDOLL_PARTS];
	}RAGDOLL;

	// registers and returns pointer to ragdoll structure
	RAGDOLL *ragdoll_register(ENTITY *ent);

	// initializes all ragdoll settings (f.e. joint limits etc)
	void ragdoll_init(RAGDOLL *ragdoll);
	
	// remove existing ragdoll and everything related to it
	// mainly used while game running, on removing npc etc
	void ragdoll_remove(ENTITY *ent);
	
	// blend given body part with the given bone (for animation blending)
	void ragdoll_blendpose(ENTITY *limb, ENTITY *actor, STRING *bone_str);
	
	// register bodypart in the physics engine
	void ragdoll_setup_bodypart(ENTITY *ent, RAGDOLL *ragdoll, var id);
	
	// create hinge joint
	void ragdoll_create_hinge(ENTITY *ent1, ENTITY *ent2, VECTOR *joint_axis, VECTOR *joint_limits, var vertex);
	
	// create 6d joint
	void ragdoll_create_6d(ENTITY *ent1, ENTITY *ent2, VECTOR *swing, VECTOR *twist, VECTOR *joint_axis, var vertex);
	
	// update model's mesh with the physical bodies
	void ragdoll_update_mesh(ENTITY *hinge, STRING *part_str, ENTITY *actor);
	
	// create ragdoll for the given entity
	void ragdoll_create(ENTITY *ent);
	
	// update ragdoll for the given entity
	void ragdoll_update(ENTITY *ent);

	#include "ragdoll.c"
#endif