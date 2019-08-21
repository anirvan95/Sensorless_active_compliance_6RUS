--6RUS.lua
--By Durgesh Salunkhe and Anirvan Dutta
--www.salunkhedurgesh.com
--www.anirvandutta.com

-- Function for Matrix Multiplication
function MatMul( m1, m2 )
    if #m1[1] ~= #m2 then       -- inner matrix-dimensions must agree
        return nil
    end

    local res = {}

    for i = 1, #m1 do
        res[i] = {}
        for j = 1, #m2[1] do
            res[i][j] = 0
            for k = 1, #m2 do
                res[i][j] = res[i][j] + m1[i][k] * m2[k][j]
            end
        end
    end

    return res
end


init_x = 0.25

-- base plate


base_length = 0.15; --Length of bottom platform
theta_b = math.pi/4;
rem_angle = theta_b;
half_angle = (2*math.pi/3) - rem_angle;
base_plate_mass = 2 --in kg
base_plate_radius = base_length/(2*math.sin(rem_angle/2))
base_plate_thickness = 0.01



-- Defining rotation of Revolute Joints
--Revolute Joint 1
rangle_1 = -math.pi/6
Rx_clock1 = {{1,0,0},{0, math.cos(rangle_1), math.sin(rangle_1)},{0, -math.sin(rangle_1),math.cos(rangle_1)}};
--Revolute Joint 2
rangle_2 = math.pi/6
Rx_clock2 = {{1,0,0},{0, math.cos(rangle_2), math.sin(rangle_2)},{0, -math.sin(rangle_2),math.cos(rangle_2)}};
--Revolute Joint 3
rangle_3 = -5*math.pi/6
Rx_clock3 = {{1,0,0},{0, math.cos(rangle_3), math.sin(rangle_3)},{0, -math.sin(rangle_3),math.cos(rangle_3)}};
--Revolute Joint 4
rangle_4 = -math.pi/2
Rx_clock4 = {{1,0,0},{0, math.cos(rangle_4), math.sin(rangle_4)},{0, -math.sin(rangle_4),math.cos(rangle_4)}};
--Revolute Joint 5
rangle_5 = math.pi/2
Rx_clock5 = {{1,0,0},{0, math.cos(rangle_5), math.sin(rangle_5)},{0, -math.sin(rangle_5),math.cos(rangle_5)}};
--Revolute Joint 5
rangle_6 = 5*math.pi/6
Rx_clock6 = {{1,0,0},{0, math.cos(rangle_6), math.sin(rangle_6)},{0, -math.sin(rangle_6),math.cos(rangle_6)}};


--base_point 1
base_point_1x = 0
base_point_1y = -base_length/(2*math.tan(rem_angle*0.5));
base_point_1z = -base_length*0.5

--base_point 2
base_point_2x = 0
base_point_2y = base_point_1y
base_point_2z = -base_point_1z;

--base_point 3
base_point_3x = 0
base_point_3y = -base_plate_radius*math.cos(half_angle+rem_angle*0.5)
base_point_3z = base_plate_radius*math.sin(half_angle+rem_angle*0.5)

--base_point 4
base_point_4x = 0
base_point_4y = base_plate_radius*math.cos(half_angle*0.5)
base_point_4z = base_plate_radius*math.sin(half_angle*0.5)

--base_point 5
base_point_5x = 0
base_point_5y = base_point_4y
base_point_5z = -base_point_4z

--base_point 6
base_point_6x = 0
base_point_6y = base_point_3y
base_point_6z = -base_point_3z

-- end-effector plate

top_length = 0.12 --Length of top platform
theta_p = 1.3089 -- 75 degrees in radians         
half_angle = theta_p
rem_angle = 2*math.pi/3 - half_angle
top_plate_mass = 1
top_plate_radius = top_length/(2*math.sin((2*math.pi/3-theta_p)/2))
top_plate_thickness = 0.01



--top_point 1
top_point_1x = init_x;
top_point_1y = -top_length/(2*math.tan(rem_angle*0.5))
top_point_1z = -top_length*0.5

--top_point 2
top_point_2x = init_x
top_point_2y = top_point_1y
top_point_2z = -top_point_1z;

--top_point 3
top_point_3x = init_x
top_point_3y = -top_plate_radius*math.cos(half_angle+rem_angle*0.5)
top_point_3z = top_plate_radius*math.sin(half_angle+rem_angle*0.5)

--top_point 4
top_point_4x = init_x
top_point_4y = top_plate_radius*math.cos(half_angle*0.5)
top_point_4z = top_plate_radius*math.sin(half_angle*0.5)

--top_point 5
top_point_5x = init_x
top_point_5y = top_point_4y
top_point_5z = -top_point_4z

--top_point 6
top_point_6x = init_x
top_point_6y = top_point_3y
top_point_6z = -top_point_3z

Ujoint_radius = 0.03
Ujoint_mass = 0

Sjoint_radius = 0.03
Sjoint_mass = 0

R_joint_thickness = 0.02
R_joint_radius = 0.02
R_joint_mass = 0

l1 = 0.12 -- length of first link
l2 = 0.24 -- length of second link

link1_mass = 0.0001
link2_mass = 0.0001

theta11 = 0
theta12 = 0
theta13 = 0
theta14 = 0
theta15 = 0
theta16 = 0
Rz_clock11 = {{math.cos(theta11),math.sin(theta11),0},{-math.sin(theta11),math.cos(theta11),0},{0,0,1}}
Rz_clock12 = {{math.cos(theta12),math.sin(theta12),0},{-math.sin(theta12),math.cos(theta12),0},{0,0,1}}
Ry_clock13 = {{math.cos(theta13),0,-math.sin(theta13)},{0,1,0},{math.sin(theta13),0,math.cos(theta13)}}
Rz_clock14 = {{math.cos(theta14),math.sin(theta14),0},{-math.sin(theta14),math.cos(theta14),0},{0,0,1}}
Ry_clock15 = {{math.cos(theta15),0,-math.sin(theta15)},{0,1,0},{math.sin(theta15),0,math.cos(theta15)}}
Rx_clock16 = {{1,0,0},{0,math.cos(theta16),math.sin(theta16)},{0,-math.sin(theta16),math.cos(theta16)}}
temp_rot = MatMul(Ry_clock15,Rx_clock16)
final_sph_rot =  MatMul(Rz_clock14,temp_rot)

theta21 = 0
theta22 = 0
theta23 = 0
Rz_clock21 = {{math.cos(theta21),math.sin(theta21),0},{-math.sin(theta21),math.cos(theta21),0},{0,0,1}}
Rz_clock22 = {{math.cos(theta22),math.sin(theta22),0},{-math.sin(theta22),math.cos(theta22),0},{0,0,1}}
Ry_clock23 = {{math.cos(theta23),0,-math.sin(theta23)},{0,1,0},{math.sin(theta23),0,math.cos(theta23)}}

theta31 = 0
theta32 = 0
theta33 = 0
Rz_clock31 = {{math.cos(theta31),math.sin(theta31),0},{-math.sin(theta31),math.cos(theta31),0},{0,0,1}}
Rz_clock32 = {{math.cos(theta32),math.sin(theta32),0},{-math.sin(theta32),math.cos(theta32),0},{0,0,1}}
Ry_clock33 = {{math.cos(theta33),0,-math.sin(theta33)},{0,1,0},{math.sin(theta33),0,math.cos(theta33)}}

theta41 = 0
theta42 = 0
theta43 = 0
Rz_clock41 = {{math.cos(theta41),math.sin(theta41),0},{-math.sin(theta41),math.cos(theta41),0},{0,0,1}}
Rz_clock42 = {{math.cos(theta42),math.sin(theta42),0},{-math.sin(theta42),math.cos(theta42),0},{0,0,1}}
Ry_clock43 = {{math.cos(theta43),0,-math.sin(theta43)},{0,1,0},{math.sin(theta43),0,math.cos(theta43)}}

theta51 = 0
theta52 = 0
theta53 = 0
Rz_clock51 = {{math.cos(theta51),math.sin(theta51),0},{-math.sin(theta51),math.cos(theta51),0},{0,0,1}}
Rz_clock52 = {{math.cos(theta52),math.sin(theta52),0},{-math.sin(theta52),math.cos(theta52),0},{0,0,1}}
Ry_clock53 = {{math.cos(theta53),0,-math.sin(theta53)},{0,1,0},{math.sin(theta53),0,math.cos(theta53)}}

theta61 = 0
theta62 = 0
theta63 = 0
Rz_clock61 = {{math.cos(theta61),math.sin(theta61),0},{-math.sin(theta61),math.cos(theta61),0},{0,0,1}}
Rz_clock62 = {{math.cos(theta62),math.sin(theta62),0},{-math.sin(theta62),math.cos(theta62),0},{0,0,1}}
Ry_clock63 = {{math.cos(theta63),0,-math.sin(theta63)},{0,1,0},{math.sin(theta63),0,math.cos(theta63)}}



bodies = {

  plate = {
    mass = base_plate_mass,
    com = {0, 0, 0},
    inertia = {
      {0.0003, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
    },
  },
  
  end_plate = {
    mass = top_plate_mass,
    com = {top_plate_radius, 0, 0},
    inertia = {
      {0, 0, 0},
      {0, 0.0003, 0},
      {0, 0, 0},
    },
  },
  
  link1 = {
    mass = link1_mass,
    com = {l1/2,0,0},
    inertia = {
      {0.0001, 0, 0},
      {0, 0, 0},
      {0, 0, 0.0001},
    },
  },
  link2 = {
    mass = link2_mass,
    com = {l2/2,0,0},
    inertia = {
      {0.0001, 0, 0},
      {0, 0, 0},
      {0, 0, 0.0001},
    },
  },
  Rjoint_cyl = {
    mass = R_joint_mass,
    com = {R_joint_thickness/2, 0, 0},
    inertia = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
    },
  },
  Sjoint_ball = {
    mass = R_joint_mass,
    com = {0, 0, 0},
    inertia = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
    },
  },
}
joints = {
  universal_zy = {
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
  },
  fixed = {},
  revolute_x = {
    {1, 0, 0, 0, 0, 0},
  },
  revolute_z = {
    {0, 0, 1, 0, 0, 0},
  },
    revolute_y = {
    {0, 1, 0, 0, 0, 0},
  },
  spherical_xyz = {
		{ 1., 0., 0., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 0., 0., 1., 0., 0., 0.}
  },
}

meshes = {
  plate = {
    name = 'plate',
    dimensions = {2*base_plate_radius,2*base_plate_radius,base_plate_thickness},
    rotate = { axis = {0.,1.,0.}, angle = 90.},
    color = {1, 1, 0},
    src = 'test_base.obj',
    mesh_center = {0, 0, 0},
  },
  Ujoint_ball = {
    name = 'Ujoint_ball',
    dimensions = {Ujoint_radius, Ujoint_radius, Ujoint_radius},
    color = {1, 1, 0},
    src = 'unit_sphere_medres.obj',
    mesh_center = {0, 0, 0},
  },
  Sjoint_ball = {
    name = 'Sjoint_ball',
    dimensions = {Sjoint_radius, Sjoint_radius, Sjoint_radius},
    color = {1, 1, 0},
    src = 'unit_sphere_medres.obj',
    mesh_center = {0, 0, 0},
  },
   Sjoint_x = {
    name = 'Sjoint_x',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {0.,0.,1.}, angle = 90.},
    color = {1, 1, 0},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
   Sjoint_z = {
    name = 'Sjoint_z',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {1.,0.,0.}, angle = 90.},
    color = {1, 1, 0},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
   Sjoint_y = {
    name = 'Sjoint_y',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {0.,0.,1.}, angle = 0.},
    color = {1, 1, 0},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
  Rjoint_cyl = {
    name = 'Rjoint_cyl',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {1.,0.,0.}, angle = 90.},
    color = {0, 1, 0},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
 Rjoint_cylf = {
    name = 'Rjoint_cylf',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {1.,0.,0.}, angle = 90.},
    color = {1, 0, 0},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
  
  Rjoint_cylse = {
    name = 'Rjoint_cylse',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {1.,0.,0.}, angle = 90.},
    color = {1, 1, 1},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
  
  Ujoint_y = {
    name = 'Ujoint_y',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {0.,0.,1.}, angle = 0.},
    color = {0, 0, 1},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
   Ujoint_z = {
    name = 'Ujoint_z',
    dimensions = {2*R_joint_radius,R_joint_thickness,2*R_joint_radius},
    rotate = { axis = {1.,0.,0.}, angle = 90.},
    color = {0, 0, 1},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
  Rjoint_link1 = {
    dimensions = {0.5*R_joint_radius, l1, 0.5*R_joint_radius},
    rotate = { axis = {0.,0.,1.}, angle = 90.},
    name = 'Rjoint_link1',
    color = {0, 1, 1},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
  Ujoint_link2 = {
    dimensions = {0.5*R_joint_radius, l2, 0.5*R_joint_radius},
    rotate = { axis = {0.,0.,1.}, angle = 90.},
    name = 'Ujoint_link2',
    color = {1, 0, 0},
    src = 'unit_cylinder_medres.obj',
    mesh_center = {0, 0, 0},
  },
  top = {
    name = 'top',
    dimensions = { 2*top_plate_radius,2*top_plate_radius, top_plate_thickness},
    rotate = { axis = {1.,0.,0.}, angle = 90.},
    color = {0.5, 1, 0.5},
    src = 'test_base.obj',
    mesh_center = {0, 0, 0},
  },
    dummy_top = {
    name = 'dummy_top',
    dimensions = {2*top_plate_radius,top_plate_thickness, 2*top_plate_radius},
    color = {0, 1, 0.5},
    src = 'test_base.obj',
    mesh_center = {0, 0, 0},
  },
}

model = {

  gravity = {-9.81,0, 0},

  frames = {

-- base-frame/-body
    {
      name = 'base',
      parent = 'ROOT',
      body = bodies.plate,
      joint = joints.fixed,
      joint_frame = {
        r = {0, 0, 0},
      },
      visuals = { meshes.plate },
    },
    -- Defining the revolute joints {The green cylinders in the visualization}
    --first joint
    {
  		name = "Rjoint_cyl1",
  		parent = "base",
  		body = bodies.Rjoint_cyl, -- the first joint is colored red for identification
  		joint = joints.revolute_z,
  		joint_frame = {
  			r = {base_point_1x, base_point_1y, base_point_1z},
        --E = MatMul(Rx_clock11,Rz_clock1)
        E = MatMul(Rz_clock11,Rx_clock1)
  		},
      visuals = { meshes.Rjoint_cylf },
  	},
    
    {
      name = 'link11',
      parent = 'Rjoint_cyl1',
      body = bodies.link1,
      joint = joints.fixed,
      joint_frame = {
        r = { l1/2,0, 0},
			},
      visuals = { meshes.Rjoint_link1 },
    },
    
    {
  		name = "Ujoint_z1",
  		parent = "link11",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = { l1/2,0., 0.},
        E = Rz_clock12
  		},
      visuals = { meshes.Ujoint_z },
  	},
    {
  		name = "Ujoint_y1",
  		parent = "Ujoint_z1",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock13
  		},
      visuals = { meshes.Ujoint_y },
  	},
    
    {
      name = 'link12',
      parent = 'Ujoint_y1',
      body = bodies.link2,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
			},
      visuals = { meshes.Ujoint_link2 },
    },
    
    {
  		name = "Sjoint_z1",
  		parent = "link12",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = {l2/2, 0., 0},
        E = Rz_clock14
  		},
      visuals = { meshes.Sjoint_z },
  	},
    
     {
  		name = "Sjoint_y1",
  		parent = "Sjoint_z1",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock15
  		},
      visuals = { meshes.Sjoint_y },
  	},
   {
  		name = "Sjoint_x1",
  		parent = "Sjoint_y1",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_x,
  		joint_frame = {
        r = {0., 0, 0},
        E = Rx_clock16
  		},
      visuals = { meshes.Sjoint_x },
  	},
    
    {
      name = 'top',
      parent = 'Sjoint_x1',
      body = bodies.plate,
      joint = joints.fixed,
      joint_frame = {
        r = {top_plate_radius, 0., 0},
      },
      visuals = { meshes.top },
    },
    
        
    --second joint
    {
  		name = "Rjoint_cyl2",
  		parent = "base",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
  			r = {base_point_2x, base_point_2y, base_point_2z},
        --E = MatMul(Rx_clock21,Rz_anti2)
        E = MatMul(Rz_clock21,Rx_clock2)
  		},
      visuals = { meshes.Rjoint_cylse },
  	},
    
    {
      name = 'link21',
      parent = 'Rjoint_cyl2',
      body = bodies.link1,
      joint = joints.fixed,
      joint_frame = {
        r = { l1/2,0., 0.},
			},
      visuals = { meshes.Rjoint_link1 },
    },
    
    {
  		name = "Ujoint_z2",
  		parent = "link21",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = { l1/2, 0., 0.},
        E = Rz_clock22
  		},
      visuals = { meshes.Ujoint_z },
  	},
    {
  		name = "Ujoint_y2",
  		parent = "Ujoint_z2",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock23
  		},
      visuals = { meshes.Ujoint_y },
  	},
    
    {
      name = 'link22',
      parent = 'Ujoint_y2',
      body = bodies.link2,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
			},
      visuals = { meshes.Ujoint_link2 },
    },
    
    {
      name = 'Sjoint_ball2',
      parent = 'link22',
      body = bodies.Sjoint_ball,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
        --E = final_sph_rot
			},
      visuals = { meshes.Sjoint_ball },
    },
    
    --third joint
    {
  		name = "Rjoint_cyl3",
  		parent = "base",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
  			r = {base_point_3x, base_point_3y, base_point_3z},
        --E = MatMul(Rx_clock31,Rz_anti3)
        E = MatMul(Rz_clock31,Rx_clock3)
  		},
      visuals = { meshes.Rjoint_cyl },
  	},
    
    {
      name = 'link31',
      parent = 'Rjoint_cyl3',
      body = bodies.link1,
      joint = joints.fixed,
      joint_frame = {
        r = { l1/2,0., 0.},
			},
      visuals = { meshes.Rjoint_link1 },
    },
    
    {
  		name = "Ujoint_z3",
  		parent = "link31",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = { l1/2,0., 0.},
        E = Rz_clock32
        
  		},
      visuals = { meshes.Ujoint_z },
  	},
    {
  		name = "Ujoint_y3",
  		parent = "Ujoint_z3",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock33
  		},
      visuals = { meshes.Ujoint_y },
  	},
    
    {
      name = 'link32',
      parent = 'Ujoint_y3',
      body = bodies.link2,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
			},
      visuals = { meshes.Ujoint_link2 },
    },
    
    {
      name = 'Sjoint_ball3',
      parent = 'link32',
      body = bodies.Sjoint_ball,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
        --E = final_sph_rot
			},
      visuals = { meshes.Sjoint_ball },
    },


    
    
    --fourth joint
    {
  		name = "Rjoint_cyl4",
  		parent = "base",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
  			r = {base_point_4x, base_point_4y, base_point_4z},
        --E = MatMul(Rx_clock41,Rz_anti4)
        E = MatMul(Rz_clock41,Rx_clock4)
  		},
      visuals = { meshes.Rjoint_cyl },
    }, 
    
    {
      name = 'link41',
      parent = 'Rjoint_cyl4',
      body = bodies.link1,
      joint = joints.fixed,
      joint_frame = {
        r = { l1/2,0., 0.},
			},
        visuals = { meshes.Rjoint_link1 },
    },

  
   {
  		name = "Ujoint_z4",
  		parent = "link41",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = { l1/2,0., 0.},
        E = Rz_clock42
  		},
      visuals = { meshes.Ujoint_z },
  	},
    
    {
  		name = "Ujoint_y4",
  		parent = "Ujoint_z4",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock43
  		},
      visuals = { meshes.Ujoint_y },
  	},
    
    {
      name = 'link42',
      parent = 'Ujoint_y4',
      body = bodies.link2,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
			},
      visuals = { meshes.Ujoint_link2 },
    },
    
    {
      name = 'Sjoint_ball4',
      parent = 'link42',
      body = bodies.Sjoint_ball,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
        --E = final_sph_rot
			},
      visuals = { meshes.Sjoint_ball },
    },

  --fifth joint
    {
  		name = "Rjoint_cyl5",
  		parent = "base",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
  			r = {base_point_5x, base_point_5y, base_point_5z},
        --E = MatMul(Rx_clock51,Rz_clock5)
        E = MatMul(Rz_clock51,Rx_clock5)
  		},
      visuals = { meshes.Rjoint_cyl },
  	},
    
    {
      name = 'link51',
      parent = 'Rjoint_cyl5',
      body = bodies.link1,
      joint = joints.fixed,
      joint_frame = {
        r = { l1/2,0., 0.},
			},
      visuals = { meshes.Rjoint_link1 },
    },
    
    {
  		name = "Ujoint_z5",
  		parent = "link51",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = { l1/2,0., 0.},
        E = Rz_clock52
  		},
      visuals = { meshes.Ujoint_z },
  	},
    {
  		name = "Ujoint_y5",
  		parent = "Ujoint_z5",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock53
  		},
      visuals = { meshes.Ujoint_y },
  	},
    
     {
      name = 'link52',
      parent = 'Ujoint_y5',
      body = bodies.link2,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
			},
      visuals = { meshes.Ujoint_link2 },
    },
    
    {
      name = 'Sjoint_ball5',
      parent = 'link52',
      body = bodies.Sjoint_ball,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
        --E = final_sph_rot
			},
      visuals = { meshes.Sjoint_ball },
    },
    
    --sixth joint
    {
  		name = "Rjoint_cyl6",
  		parent = "base",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
  			r = {base_point_6x, base_point_6y, base_point_6z},
        --E = MatMul(Rx_clock61,Rz_clock6)
        E = MatMul(Rz_clock61,Rx_clock6)
  		},
      visuals = { meshes.Rjoint_cyl },
  	},
    
    {
      name = 'link61',
      parent = 'Rjoint_cyl6',
      body = bodies.link1,
      joint = joints.fixed,
      joint_frame = {
        r = {l1/2,0., 0.},
			},
      visuals = { meshes.Rjoint_link1 },
    },

    {
  		name = "Ujoint_z6",
  		parent = "link61",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_z,
  		joint_frame = {
        r = { l1/2,0., 0.},
        E = Rz_clock62
  		},
      visuals = { meshes.Ujoint_z },
  	},
    {
  		name = "Ujoint_y6",
  		parent = "Ujoint_z6",
  		body = bodies.Rjoint_cyl,
  		joint = joints.revolute_y,
  		joint_frame = {
        r = {0., 0, 0},
        E = Ry_clock63
  		},
      visuals = { meshes.Ujoint_y },
  	},
    
    {
      name = 'link62',
      parent = 'Ujoint_y6',
      body = bodies.link2,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
			},
      visuals = { meshes.Ujoint_link2 },
    },

{
      name = 'Sjoint_ball6',
      parent = 'link62',
      body = bodies.Sjoint_ball,
      joint = joints.fixed,
      joint_frame = {
        r = {l2/2, 0., 0},
        --E = final_sph_rot
			},
      visuals = { meshes.Sjoint_ball },
    },
  -- Defining Top Platform
    
  },
}
return model
