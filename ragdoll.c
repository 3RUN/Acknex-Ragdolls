
// registers and returns pointer to ragdoll structure
RAGDOLL *ragdoll_register(ENTITY *ent)
{
	if(!ent)
	{
		diag("ERROR! Can't create ragdoll! Entity doesn't exist!");
		return NULL;
	}
	
	RAGDOLL *ragdoll = sys_malloc(sizeof(RAGDOLL));
	ragdoll_init(ragdoll);
	ent->OBJ_RAGDOLL_STRUCT = ragdoll;
	return ragdoll;
}

// initializes all ragdoll settings (f.e. joint limits etc)
void ragdoll_init(RAGDOLL *ragdoll)
{
	if(!ragdoll)
	{
		diag("ERROR! Can't init ragdoll! Nothing to init!");
		return NULL;
	}
	
	// set bone names
	ragdoll->bone[RD_PELVIS] = str_create(_chr(bone_pelvis_str));
	ragdoll->bone[RD_TORSO] = str_create(_chr(bone_torso_str));
	ragdoll->bone[RD_HEAD] = str_create(_chr(bone_head_str));
	ragdoll->bone[RD_ARM_L_UP] = str_create(_chr(bone_arm_l_up_str));
	ragdoll->bone[RD_ARM_L_DOWN] = str_create(_chr(bone_arm_l_down_str));
	ragdoll->bone[RD_ARM_L_PALM] = str_create(_chr(bone_arm_l_palm_str));
	ragdoll->bone[RD_ARM_R_UP] = str_create(_chr(bone_arm_r_up_str));
	ragdoll->bone[RD_ARM_R_DOWN] = str_create(_chr(bone_arm_r_down_str));
	ragdoll->bone[RD_ARM_R_PALM] = str_create(_chr(bone_arm_r_palm_str));
	ragdoll->bone[RD_LEG_L_UP] = str_create(_chr(bone_leg_l_up_str));
	ragdoll->bone[RD_LEG_L_DOWN] = str_create(_chr(bone_leg_l_down_str));
	ragdoll->bone[RD_LEG_L_HEEL] = str_create(_chr(bone_leg_l_heel_str));
	ragdoll->bone[RD_LEG_R_UP] = str_create(_chr(bone_leg_r_up_str));
	ragdoll->bone[RD_LEG_R_DOWN] = str_create(_chr(bone_leg_r_down_str));
	ragdoll->bone[RD_LEG_R_HEEL] = str_create(_chr(bone_leg_r_heel_str));
	
	// mass factor
	// I've read that it's better to attach light weight bodies to heavier ones
	// f.e. pelvis is very heavy, while legs (which are attached to it) are lighter
	ragdoll->mass[RD_PELVIS] = 2; // pelvis + torso are the heaviest parts of the ragdoll
	ragdoll->mass[RD_TORSO] = 2;
	ragdoll->mass[RD_HEAD] = 0.25;
	ragdoll->mass[RD_ARM_L_UP] = 0.5;
	ragdoll->mass[RD_ARM_L_DOWN] = 0.25;
	ragdoll->mass[RD_ARM_L_PALM] = 0.15;
	ragdoll->mass[RD_ARM_R_UP] = 0.5;
	ragdoll->mass[RD_ARM_R_DOWN] = 0.25;
	ragdoll->mass[RD_ARM_R_PALM] = 0.15;
	ragdoll->mass[RD_LEG_L_UP] = 0.5;
	ragdoll->mass[RD_LEG_L_DOWN] = 0.25;
	ragdoll->mass[RD_LEG_L_HEEL] = 0.15;
	ragdoll->mass[RD_LEG_R_UP] = 0.5;
	ragdoll->mass[RD_LEG_R_DOWN] = 0.25;
	ragdoll->mass[RD_LEG_R_HEEL] = 0.15;
	
	// joint vertexes
	// vertexes used as joint anchor positions
	// each anchor is defined via ent2 vertex
	// so f.e. upper leg's anchor position can be found in pelvis
	// because leg is ent1 and pelvis is ent2 (in hinge joint function)
	ragdoll->joint_vertex[RD_TORSO] = 67;
	ragdoll->joint_vertex[RD_HEAD] = 67;
	ragdoll->joint_vertex[RD_ARM_L_UP] = 68;
	ragdoll->joint_vertex[RD_ARM_L_DOWN] = 67;
	ragdoll->joint_vertex[RD_ARM_L_PALM] = 67;
	ragdoll->joint_vertex[RD_ARM_R_UP] = 69;
	ragdoll->joint_vertex[RD_ARM_R_DOWN] = 67;
	ragdoll->joint_vertex[RD_ARM_R_PALM] = 67;
	ragdoll->joint_vertex[RD_LEG_L_UP] = 69;
	ragdoll->joint_vertex[RD_LEG_L_DOWN] = 67;
	ragdoll->joint_vertex[RD_LEG_L_HEEL] = 67;
	ragdoll->joint_vertex[RD_LEG_R_UP] = 68;
	ragdoll->joint_vertex[RD_LEG_R_DOWN] = 67;
	ragdoll->joint_vertex[RD_LEG_R_HEEL] = 67;
	
	// joint axis
	vec_set(&ragdoll->joint_axis[RD_TORSO], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_HEAD], vector(1, 0, 0));
	vec_set(&ragdoll->joint_axis[RD_ARM_L_UP], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_ARM_L_DOWN], vector(0, 0, 1));
	vec_set(&ragdoll->joint_axis[RD_ARM_L_PALM], vector(1, 0, 0));
	vec_set(&ragdoll->joint_axis[RD_ARM_R_UP], vector(0, -1, 0));
	vec_set(&ragdoll->joint_axis[RD_ARM_R_DOWN], vector(0, 0, -1));
	vec_set(&ragdoll->joint_axis[RD_ARM_R_PALM], vector(-1, 0, 0));
	vec_set(&ragdoll->joint_axis[RD_LEG_L_UP], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_LEG_L_DOWN], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_LEG_L_HEEL], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_LEG_R_UP], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_LEG_R_DOWN], vector(0, 1, 0));
	vec_set(&ragdoll->joint_axis[RD_LEG_R_HEEL], vector(0, 1, 0));
	
	// joint limits
	vec_set(&ragdoll->joint_limit[RD_TORSO], vector(-75, 35, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_ARM_L_DOWN], vector(0, 145, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_ARM_L_PALM], vector(-35, 35, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_ARM_R_DOWN], vector(0, 145, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_ARM_R_PALM], vector(-35, 35, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_LEG_L_UP], vector(-15, 90, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_LEG_L_DOWN], vector(-90, 0, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_LEG_L_HEEL], vector(-45, 15, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_LEG_R_UP], vector(-15, 90, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_LEG_R_DOWN], vector(-90, 0, RAGDOLL_BONE_RESTITUTION));
	vec_set(&ragdoll->joint_limit[RD_LEG_R_HEEL], vector(-45, 15, RAGDOLL_BONE_RESTITUTION));
}

// remove existing ragdoll and everything related to it
// mainly used while game running, on removing npc etc
void ragdoll_remove(ENTITY *ent)
{
	if(!ent)
	{
		diag("ERROR! Can't remove ragdoll struct! Nothing to remove!");
		return NULL;
	}
	
	// get ragdoll pointer
	RAGDOLL *ragdoll = ent->OBJ_RAGDOLL_STRUCT;
	
	int i = 0;
	for(i = 0; i < MAX_RAGDOLL_PARTS; i++)
	{
		str_remove(ragdoll->bone[i]);
		
		// we don't need to unregister physical entities here
		// or remove the joints, it's all done automatically in ackphysX header
		// in physX_ent_remove function
		if(ragdoll->rg_part[i])
		{
			ragdoll->rg_part[i]->OBJ_TYPE = TYPE_NONE;
			ptr_remove(ragdoll->rg_part[i]);
			ragdoll->rg_part[i] = NULL;
		}
	}
	
	if(ragdoll->ghost_ent)
	{
		ptr_remove(ragdoll->ghost_ent);
		ragdoll->ghost_ent = NULL;
	}
	
	sys_free(ragdoll);
	ent->OBJ_RAGDOLL_STRUCT = 0;
	ent->OBJ_HAS_RAGDOLL = false;
}

// blend given body part with the given bone (for animation blending)
void ragdoll_blendpose(ENTITY *limb, ENTITY *actor, STRING *bone_str)
{
	VECTOR temp_pos;
	ANGLE temp_ang;
	vec_for_bone(&temp_pos, actor, bone_str);
	ang_for_bone(&temp_ang, actor, bone_str);
	
	vec_set(&limb->x, &temp_pos);
	vec_set(&limb->pan, &temp_ang);
	
	pXent_setposition(limb, &temp_pos);
	pXent_rotate(limb, nullvector, &temp_ang);
}

// register bodypart in the physics engine
void ragdoll_setup_bodypart(ENTITY *ent, RAGDOLL *ragdoll, var id)
{
	#ifndef DEBUG_RD_BODY_PARTS
		set(ent, PASSABLE | INVISIBLE);
		ent->OBJ_TYPE = TYPE_RD_BODY_PART;
		pXent_settype(ent, PH_RIGID, PH_CONVEX);
		pXent_setgroup(ent, GROUP_RAGDOLL + id);
		pXent_setfriction(ent, RAGDOLL_FRICTION);
		pXent_setelasticity(ent, RAGDOLL_RESTITUTION);
		pXent_setdamping(ent, RAGDOLL_LINEAR_DAMPING, RAGDOLL_ANGULAR_DAMPING);
		pXent_setmass(ent, pXent_getmass(ent) * ragdoll->mass[id]);
		pXent_setiterations(ent, RAGDOLL_ITERATION);
		pXent_setccdskeleton(ent, nullvector, true);
	#endif
}

// create hinge joint
void ragdoll_create_hinge(ENTITY *ent1, ENTITY *ent2, VECTOR *joint_axis, VECTOR *joint_limits, var vertex)
{
	#ifndef DEBUG_RD_BODY_PARTS
		VECTOR temp_pos;
		vec_for_vertex(&temp_pos, ent2, vertex);
		pXcon_add(PH_HINGE, ent1, ent2, false);
		pXcon_setparams1(ent1, &temp_pos, joint_axis, NULL);
		pXcon_setparams2(ent1, joint_limits, NULL, NULL);
	#endif
}

// create 6d joint
void ragdoll_create_6d(ENTITY *ent1, ENTITY *ent2, VECTOR *swing, VECTOR *twist, VECTOR *joint_axis, var vertex)
{
	#ifndef DEBUG_RD_BODY_PARTS
		VECTOR temp_pos;
		vec_for_vertex(&temp_pos, ent2, vertex);
		pXcon_add(PH_6DJOINT, ent1, ent2, false);
		pXcon_setparams1(ent1, &temp_pos, joint_axis, NULL);
		pXcon_setparams2(ent1, swing, twist, vector(RAGDOLL_6D_SPRING, RAGDOLL_6D_DAMPING, RAGDOLL_6D_LINEAR_LIMIT));
		pXcon_set6djoint(ent1, joint_motion, NULL);	
	#endif
}

// update model's mesh with the physical bodies
void ragdoll_update_mesh(ENTITY *hinge, STRING *part_str, ENTITY *actor)
{
	VECTOR temp_vec[4];
	
	vec_fill(&temp_vec[0], 0);
	vec_fill(&temp_vec[1], 0);
	vec_fill(&temp_vec[2], 0);
	vec_fill(&temp_vec[3], 0);
	
	ent_bonereset(actor, part_str);
	
	vec_for_bone(&temp_vec[0], actor, part_str);
	ang_for_bone(&temp_vec[1], actor, part_str);
	
	vec_set(&temp_vec[2], &hinge->x);
	vec_sub(&temp_vec[2], &temp_vec[0]);
	
	temp_vec[2].x /= actor->scale_x; 
	temp_vec[2].y /= actor->scale_y; 
	temp_vec[2].z /= actor->scale_z;
	
	vec_rotateback(&temp_vec[2], &temp_vec[1]);
	temp_vec[1].x = 360 - temp_vec[1].x; 
	temp_vec[1].y = 360 - temp_vec[1].y; 
	temp_vec[1].z = 360 - temp_vec[1].z;
	
	vec_set(&temp_vec[3], nullvector);
	ang_add(&temp_vec[3], vector(temp_vec[1].x, 0, 0));
	ang_add(&temp_vec[3], vector(0, temp_vec[1].y, 0));
	ang_add(&temp_vec[3], vector(0, 0, temp_vec[1].z));
	
	ang_rotate(&temp_vec[3], &hinge->pan);
	ent_bonerotate(actor, part_str, &temp_vec[3]);
	ent_bonemove(actor, part_str, &temp_vec[2]);
}

// create ragdoll for the given entity
void ragdoll_create(ENTITY *ent)
{
	if(!ent)
	{
		diag("ERROR! Can't create ragdoll! Entity doesn't exist!");
		return NULL;
	}
	
	if(ent->OBJ_HAS_RAGDOLL == true)
	{
		diag("ERROR! Can't create ragdoll! It already exists!");
		return NULL;
	}
	
	RAGDOLL *ragdoll = ragdoll_register(ent);
	
	// create ghost entity
	ragdoll->ghost_ent = ent_create(human_mdl, &ent->x, NULL);
	set(ragdoll->ghost_ent, PASSABLE | INVISIBLE);
	vec_set(&ragdoll->ghost_ent->scale_x, &ent->scale_x);
	vec_set(&ragdoll->ghost_ent->pan, &ent->pan);
	
	// create each body part !
	// pelvis
	ragdoll->rg_part[RD_PELVIS] = ent_create(rg_pelvis_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_PELVIS], ragdoll->ghost_ent, bone_pelvis_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_PELVIS], ragdoll, RD_PELVIS);
	
	// torso
	ragdoll->rg_part[RD_TORSO] = ent_create(rg_torso_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_TORSO], ragdoll->ghost_ent, bone_torso_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_TORSO], ragdoll, RD_TORSO);
	ragdoll_create_hinge(ragdoll->rg_part[RD_TORSO], ragdoll->rg_part[RD_PELVIS], &ragdoll->joint_axis[RD_TORSO], &ragdoll->joint_limit[RD_TORSO], ragdoll->joint_vertex[RD_TORSO]);
	
	// head
	ragdoll->rg_part[RD_HEAD] = ent_create(rg_head_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_HEAD], ragdoll->ghost_ent, bone_head_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_HEAD], ragdoll, RD_HEAD);
	ragdoll_create_6d(ragdoll->rg_part[RD_HEAD], ragdoll->rg_part[RD_TORSO], swing_head_ang, twist_head_ang, &ragdoll->joint_axis[RD_HEAD], ragdoll->joint_vertex[RD_HEAD]);
	
	// upper left arm
	ragdoll->rg_part[RD_ARM_L_UP] = ent_create(rg_arm_l_up_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_ARM_L_UP], ragdoll->ghost_ent, bone_arm_l_up_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_ARM_L_UP], ragdoll, RD_ARM_L_UP);
	ragdoll_create_6d(ragdoll->rg_part[RD_ARM_L_UP], ragdoll->rg_part[RD_TORSO], swing_arm_ang, twist_arm_ang, &ragdoll->joint_axis[RD_ARM_L_UP], ragdoll->joint_vertex[RD_ARM_L_UP]);
	
	// lower left arm
	ragdoll->rg_part[RD_ARM_L_DOWN] = ent_create(rg_arm_l_down_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_ARM_L_DOWN], ragdoll->ghost_ent, bone_arm_l_down_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_ARM_L_DOWN], ragdoll, RD_ARM_L_DOWN);
	ragdoll_create_hinge(ragdoll->rg_part[RD_ARM_L_DOWN], ragdoll->rg_part[RD_ARM_L_UP], &ragdoll->joint_axis[RD_ARM_L_DOWN], &ragdoll->joint_limit[RD_ARM_L_DOWN], ragdoll->joint_vertex[RD_ARM_L_DOWN]);
	
	// left palm
	ragdoll->rg_part[RD_ARM_L_PALM] = ent_create(rg_arm_l_palm_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_ARM_L_PALM], ragdoll->ghost_ent, bone_arm_l_palm_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_ARM_L_PALM], ragdoll, RD_ARM_L_PALM);
	ragdoll_create_hinge(ragdoll->rg_part[RD_ARM_L_PALM], ragdoll->rg_part[RD_ARM_L_DOWN], &ragdoll->joint_axis[RD_ARM_L_PALM], &ragdoll->joint_limit[RD_ARM_L_PALM], ragdoll->joint_vertex[RD_ARM_L_PALM]);
	
	// upper right arm
	ragdoll->rg_part[RD_ARM_R_UP] = ent_create(rg_arm_r_up_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_ARM_R_UP], ragdoll->ghost_ent, bone_arm_r_up_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_ARM_R_UP], ragdoll, RD_ARM_R_UP);
	ragdoll_create_6d(ragdoll->rg_part[RD_ARM_R_UP], ragdoll->rg_part[RD_TORSO], swing_arm_ang, twist_arm_ang, &ragdoll->joint_axis[RD_ARM_R_UP], ragdoll->joint_vertex[RD_ARM_R_UP]);
	
	// lower right arm
	ragdoll->rg_part[RD_ARM_R_DOWN] = ent_create(rg_arm_r_down_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_ARM_R_DOWN], ragdoll->ghost_ent, bone_arm_r_down_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_ARM_R_DOWN], ragdoll, RD_ARM_R_DOWN);
	ragdoll_create_hinge(ragdoll->rg_part[RD_ARM_R_DOWN], ragdoll->rg_part[RD_ARM_R_UP], &ragdoll->joint_axis[RD_ARM_R_DOWN], &ragdoll->joint_limit[RD_ARM_R_DOWN], ragdoll->joint_vertex[RD_ARM_R_DOWN]);
	
	// right palm
	ragdoll->rg_part[RD_ARM_R_PALM] = ent_create(rg_arm_r_palm_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_ARM_R_PALM], ragdoll->ghost_ent, bone_arm_r_palm_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_ARM_R_PALM], ragdoll, RD_ARM_R_PALM);
	ragdoll_create_hinge(ragdoll->rg_part[RD_ARM_R_PALM], ragdoll->rg_part[RD_ARM_R_DOWN], &ragdoll->joint_axis[RD_ARM_R_PALM], &ragdoll->joint_limit[RD_ARM_R_PALM], ragdoll->joint_vertex[RD_ARM_R_PALM]);
	
	// left upper leg
	ragdoll->rg_part[RD_LEG_L_UP] = ent_create(rg_leg_l_up_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_LEG_L_UP], ragdoll->ghost_ent, bone_leg_l_up_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_LEG_L_UP], ragdoll, RD_LEG_L_UP);
	ragdoll_create_hinge(ragdoll->rg_part[RD_LEG_L_UP], ragdoll->rg_part[RD_PELVIS], &ragdoll->joint_axis[RD_LEG_L_UP], &ragdoll->joint_limit[RD_LEG_L_UP], ragdoll->joint_vertex[RD_LEG_L_UP]);
	
	// left lower leg
	ragdoll->rg_part[RD_LEG_L_DOWN] = ent_create(rg_leg_l_down_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_LEG_L_DOWN], ragdoll->ghost_ent, bone_leg_l_down_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_LEG_L_DOWN], ragdoll, RD_LEG_L_DOWN);
	ragdoll_create_hinge(ragdoll->rg_part[RD_LEG_L_DOWN], ragdoll->rg_part[RD_LEG_L_UP], &ragdoll->joint_axis[RD_LEG_L_DOWN], &ragdoll->joint_limit[RD_LEG_L_DOWN], ragdoll->joint_vertex[RD_LEG_L_DOWN]);
	
	// left heel
	ragdoll->rg_part[RD_LEG_L_HEEL] = ent_create(rg_leg_l_heel_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_LEG_L_HEEL], ragdoll->ghost_ent, bone_leg_l_heel_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_LEG_L_HEEL], ragdoll, RD_LEG_L_HEEL);
	ragdoll_create_hinge(ragdoll->rg_part[RD_LEG_L_HEEL], ragdoll->rg_part[RD_LEG_L_DOWN], &ragdoll->joint_axis[RD_LEG_L_HEEL], &ragdoll->joint_limit[RD_LEG_L_HEEL], ragdoll->joint_vertex[RD_LEG_L_HEEL]);
	
	// right upper leg
	ragdoll->rg_part[RD_LEG_R_UP] = ent_create(rg_leg_r_up_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_LEG_R_UP], ragdoll->ghost_ent, bone_leg_r_up_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_LEG_R_UP], ragdoll, RD_LEG_R_UP);
	ragdoll_create_hinge(ragdoll->rg_part[RD_LEG_R_UP], ragdoll->rg_part[RD_PELVIS], &ragdoll->joint_axis[RD_LEG_R_UP], &ragdoll->joint_limit[RD_LEG_R_UP], ragdoll->joint_vertex[RD_LEG_R_UP]);
	
	// right lower leg
	ragdoll->rg_part[RD_LEG_R_DOWN] = ent_create(rg_leg_r_down_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_LEG_R_DOWN], ragdoll->ghost_ent, bone_leg_r_down_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_LEG_R_DOWN], ragdoll, RD_LEG_R_DOWN);
	ragdoll_create_hinge(ragdoll->rg_part[RD_LEG_R_DOWN], ragdoll->rg_part[RD_LEG_R_UP], &ragdoll->joint_axis[RD_LEG_R_DOWN], &ragdoll->joint_limit[RD_LEG_R_DOWN], ragdoll->joint_vertex[RD_LEG_R_DOWN]);
	
	// right heel
	ragdoll->rg_part[RD_LEG_R_HEEL] = ent_create(rg_leg_r_heel_mdl, &ent->x, NULL);
	ragdoll_blendpose(ragdoll->rg_part[RD_LEG_R_HEEL], ragdoll->ghost_ent, bone_leg_r_heel_str);
	ragdoll_setup_bodypart(ragdoll->rg_part[RD_LEG_R_HEEL], ragdoll, RD_LEG_R_HEEL);
	ragdoll_create_hinge(ragdoll->rg_part[RD_LEG_R_HEEL], ragdoll->rg_part[RD_LEG_R_DOWN], &ragdoll->joint_axis[RD_LEG_R_HEEL], &ragdoll->joint_limit[RD_LEG_R_HEEL], ragdoll->joint_vertex[RD_LEG_R_HEEL]);
	
	// now blend to the animated model's bone ang/pos
	int i = 0;
	for(i = 0; i < MAX_RAGDOLL_PARTS; i++)
	{
		ragdoll_blendpose(ragdoll->rg_part[i], ent, ragdoll->bone[i]);
	}
	
	// update the mesh now
	ent->OBJ_HAS_RAGDOLL = true;
	
	// remove ghost entity, because we don't need it anymore
	ptr_remove(ragdoll->ghost_ent);
	ragdoll->ghost_ent = NULL;
}

// update ragdoll for the given entity
void ragdoll_update(ENTITY *ent)
{
	if(!ent)
	{
		diag("ERROR! Can't update ragdoll! Entity doesn't exist!");
		return NULL;
	}
	
	if(ent->OBJ_HAS_RAGDOLL == false)
	{
		diag("ERROR! Can't update ragdoll! Nothing to update!");
		return NULL;
	}

	#ifndef DEBUG_RD_BODY_PARTS
		// get ragdoll pointer
		RAGDOLL *ragdoll = ent->OBJ_RAGDOLL_STRUCT;
		
		// move/rotate model with pelvis body part
		vec_set(&ent->x, &ragdoll->rg_part[RD_PELVIS]->x);
		vec_set(&ent->pan, &ragdoll->rg_part[RD_PELVIS]->pan);
		
		// update all physics (limb) parts
		int i = 0;
		for(i = 0; i < MAX_RAGDOLL_PARTS; i++)
		{
			ragdoll_update_mesh(ragdoll->rg_part[i], ragdoll->bone[i], ent);
		}
	#endif
}