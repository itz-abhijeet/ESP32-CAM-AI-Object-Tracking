/*

  Main printable options:
    assembly
    base_plate
    pan_servo_cage
    pan_horn_disk
    tilt_yoke_base
    camera_cradle
    servo_holder
*/

$fn = 72;

PART = "assembly";

servo_holder_file = "../stl/servo_holder.stl";


m25_clear = 2.8;
m3_clear = 3.4;

cam_w = 27.0;
cam_h = 40.0;
cam_t = 19.0;

cam_clear_w = cam_w + 1.4;
cam_clear_h = cam_h + 1.4;
cam_clear_t = cam_t + 1.8;

base_plate_x = 96;
base_plate_y = 78;
base_plate_h = 4.0;
base_plate_r = 8.0;

cage_h = 39;
servo_center_z = 15.8;
ear_shelf_x = 9.5;
ear_shelf_y = 18.0;
ear_shelf_h = 3.2;

pan_disk_d = 43;
pan_disk_h = 5.0;
pan_screw_circle = 30;

yoke_base_h = 5;

yoke_inner_w = 36;
yoke_arm_t = 5.5;
yoke_arm_h = 49;
yoke_axis_z = 34;

left_arm_x = -(yoke_inner_w/2 + yoke_arm_t/2);
right_arm_x =  (yoke_inner_w/2 + yoke_arm_t/2);


cradle_w = cam_clear_w + 5.0;
cradle_t = cam_clear_t + 2.2;

module rounded_box(size=[10,10,10], r=2, center=false) {
    x=size[0]; y=size[1]; z=size[2];
    rr = min(r, min(x,y)/2 - 0.01);
    translate(center ? [-x/2,-y/2,-z/2] : [0,0,0])
        hull() {
            for (ix=[rr, x-rr])
                for (iy=[rr, y-rr])
                    translate([ix,iy,0]) cylinder(h=z, r=rr);
        }
}

module cyl_x(d,l) { rotate([0,90,0]) cylinder(d=d, h=l, center=true); }

module cyl_y(d,l) { rotate([90,0,0]) cylinder(d=d, h=l, center=true); }

module rounded_panel_y(w,h,depth,r) {
    union() {
        cube([max(0.01,w-2*r), depth, h], center=true);
        cube([w, depth, max(0.01,h-2*r)], center=true);
        for (sx=[-1,1])
            for (sz=[-1,1])
                translate([sx*(w/2-r),0,sz*(h/2-r)]) cyl_y(2*r, depth);
    }
}

module rounded_panel_x(w_y,h_z,depth_x,r) {
    union() {
        cube([depth_x, max(0.01,w_y-2*r), h_z], center=true);
        cube([depth_x, w_y, max(0.01,h_z-2*r)], center=true);
        for (sy=[-1,1])
            for (sz=[-1,1])
                translate([0,sy*(w_y/2-r),sz*(h_z/2-r)]) cyl_x(2*r, depth_x);
    }
}

module screw_circle_z(radius=15, d=2.8, h=8) {
    for (a=[45,135,225,315])
        rotate([0,0,a])
            translate([radius,0,0])
                cylinder(d=d, h=h, center=true);
}

module screw_square_x(y=10,z=10,d=2.4,depth=8) {
    for (sy=[-1,1])
        for (sz=[-1,1])
            translate([0, sy*y, sz*z]) cyl_x(d, depth);
}

module screw_square_y(x=16,z=22,d=2.4,depth=8) {
    for (sx=[-1,1])
        for (sz=[-1,1])
            translate([sx*x, 0, sz*z]) cyl_y(d, depth);
}

module bottom_base_plate_v1() {

    plate_h = 12.0;

    pan_cage_x = 72.0;
    pan_cage_y = 56.0;

    pan_cage_clearance = 0.9;

    pocket_x = pan_cage_x + pan_cage_clearance;
    pocket_y = pan_cage_y + pan_cage_clearance;

    pocket_depth = 9.0;

    bottom_floor = plate_h - pocket_depth;

    pocket_r = 3.0;

    pocket_shift_x = 0.0;
    pocket_shift_y = 0.0;

    plate_top_z = plate_h / 2;

    pocket_center_z = plate_top_z - pocket_depth / 2 + 0.01;

    difference() {
        rounded_box(
            [base_plate_x, base_plate_y, plate_h],
            r=base_plate_r,
            center=true
        );

        translate([
            pocket_shift_x,
            pocket_shift_y,
            pocket_center_z
        ])
            rounded_box(
                [pocket_x, pocket_y, pocket_depth + 0.08],
                r=pocket_r,
                center=true
            );
    }
}

module pan_servo_open_cage_v1() {

    mg_body_x = 40.0;
    mg_body_y = 20.0;
    mg_body_z = 40.5;

    mg_body_clear_x = mg_body_x + 2.8;
    mg_body_clear_y = mg_body_y + 2.6;
    mg_body_clear_z = mg_body_z + 2.5;

    mg_shaft_from_near_edge = 10;
    mg_boss_x_offset = -mg_body_x/2 + mg_shaft_from_near_edge;

    mg_boss_clear_d   = 19.0;
    mg_spline_clear_d = 10.5;

    mg_side_wall = 4.5;
    mg_back_wall = 4.5;
    mg_roof_h = 3.2;

    mg_bottom_gap = 2.5;

    mg_top_air_gap = 3.0;

    mg_servo_center_z = mg_bottom_gap + mg_body_clear_z/2;

    mg_cage_x = 72.0;
    mg_cage_y = 56.0;
    mg_cage_h = mg_bottom_gap + mg_body_clear_z + mg_top_air_gap + mg_roof_h;

    mg_mount_x = 29.0;
    mg_mount_y = 21.0;
    mg_mount_hole_d = 3.4;

    front_open_y = mg_cage_y/2 + 8.0;

    left_inner_x  = -mg_cage_x/2 + mg_side_wall;
    right_inner_x =  mg_cage_x/2 - mg_side_wall;

    wire_side = -1;

    wire_exit_y = 0.0;
    wire_exit_z = 10.0;
    wire_exit_w = 20.0;
    wire_exit_h = 12.0;

    body_half_x = mg_body_clear_x/2;
    guide_protrude = (mg_cage_x/2 - mg_side_wall) - body_half_x + 2.0;
    guide_y = mg_body_clear_y + 8.0;
    guide_h = 6.5;

    guide_z_low  = mg_servo_center_z - 5.0;
    guide_z_high = mg_servo_center_z + 5;

    rear_stop_inner_y = mg_body_clear_y/2;
    rear_stop_outer_y = mg_cage_y/2 - mg_back_wall;
    rear_stop_y_len = rear_stop_outer_y - rear_stop_inner_y;
    rear_stop_y_center = rear_stop_inner_y + rear_stop_y_len/2;
    rear_stop_h = 10.0;

    difference() {
        union() {
            translate([-mg_cage_x/2 + mg_side_wall/2, 0, mg_cage_h/2])
                rounded_box(
                    [mg_side_wall, mg_cage_y, mg_cage_h],
                    r=3,
                    center=true
                );

            translate([mg_cage_x/2 - mg_side_wall/2, 0, mg_cage_h/2])
                rounded_box(
                    [mg_side_wall, mg_cage_y, mg_cage_h],
                    r=3,
                    center=true
                );

            translate([0, mg_cage_y/2 - mg_back_wall/2, mg_cage_h/2])
                rounded_box(
                    [mg_cage_x, mg_back_wall, mg_cage_h],
                    r=3,
                    center=true
                );

            translate([0, 0, mg_cage_h - mg_roof_h/2])
                rounded_box(
                    [mg_cage_x, mg_cage_y, mg_roof_h],
                    r=4,
                    center=true
                );

            for (sx=[-1,1])
                for (gz=[guide_z_low, guide_z_high])
                    translate([
                        sx * (mg_cage_x/2 - mg_side_wall - guide_protrude/2),
                        0,
                        gz
                    ])
                        rounded_box(
                            [guide_protrude, guide_y, guide_h],
                            r=0.9,
                            center=true
                        );

            translate([0, rear_stop_y_center, mg_servo_center_z])
                rounded_box(
                    [mg_body_clear_x + 12.0, rear_stop_y_len, rear_stop_h],
                    r=1.2,
                    center=true
                );
        }

        translate([0, 0, mg_servo_center_z])
            cube(
                [mg_body_clear_x, mg_body_clear_y, mg_body_clear_z],
                center=true
            );

        translate([0, -mg_cage_y/2, mg_servo_center_z])
            cube(
                [mg_body_clear_x + 7.0, front_open_y, mg_body_clear_z + 4.0],
                center=true
            );

        translate([mg_boss_x_offset, 0, mg_cage_h - mg_roof_h/2])
            cylinder(
                d=mg_boss_clear_d,
                h=mg_roof_h + 5.0,
                center=true
            );

        translate([mg_boss_x_offset, 0, mg_cage_h])
            cylinder(
                d=mg_spline_clear_d,
                h=16.0,
                center=true
            );

        for (sx=[-1,1])
            for (sy=[-1,1])
                translate([sx*mg_mount_x, sy*mg_mount_y, 3])
                    cylinder(
                        d=mg_mount_hole_d,
                        h=14,
                        center=true
                    );

        translate([
            wire_side * mg_cage_x/2,
            wire_exit_y,
            wire_exit_z
        ])
            rounded_panel_x(
                wire_exit_w,
                wire_exit_h,
                mg_side_wall + 10,
                3
            );
    }
}

module pan_horn_adapter_disk_v1() {

    local_eps = 0.02;

    disk_h = 3.4;

    adapter_disk_d = max(pan_disk_d, 58.0);

    mg_horn_recess_depth = 1.0;

    mg_horn_center_d = 21.5;

    mg_horn_arm_w = 7.8;
    mg_horn_arm_len = 44.0;

    mg_center_access_d = 4.3;

    mg_horn_hole_d_horizontal = 1.8;
    mg_horn_hole_d_vertical   = 1.8;

    mg_horn_hole_positions = [6, 10, 14, 18, 22];

    outer_screw_d = 3.2;

    neck_counterbore_d = 4.0;
    neck_counterbore_depth = 0.35;

    difference() {
        union() {
            cylinder(
                d=adapter_disk_d,
                h=disk_h,
                center=true
            );

            translate([0, 0, disk_h/2])
                cylinder(
                    d=22.0,
                    h=0.5,
                    center=true
                );

            for (a=[45,135,225,315])
                rotate([0,0,a])
                    translate([pan_screw_circle/2, 0, disk_h/2])
                        cylinder(
                            d=6.0,
                            h=0.4,
                            center=true
                        );
        }

        translate([0, 0, -disk_h/2 + mg_horn_recess_depth/2 - local_eps]) {

            cylinder(
                d=mg_horn_center_d,
                h=mg_horn_recess_depth + 0.2,
                center=true
            );

            cube(
                [mg_horn_arm_len, mg_horn_arm_w, mg_horn_recess_depth + 0.2],
                center=true
            );

            cube(
                [mg_horn_arm_w, mg_horn_arm_len, mg_horn_recess_depth + 0.2],
                center=true
            );
        }

        cylinder(
            d=mg_center_access_d,
            h=disk_h + 8,
            center=true
        );

        for (p = mg_horn_hole_positions) {
            translate([ p, 0, 0])
                cylinder(
                    d=mg_horn_hole_d_horizontal,
                    h=disk_h + 8,
                    center=true
                );

            translate([-p, 0, 0])
                cylinder(
                    d=mg_horn_hole_d_horizontal,
                    h=disk_h + 8,
                    center=true
                );
        }

        for (p = mg_horn_hole_positions) {
            translate([0,  p, 0])
                cylinder(
                    d=mg_horn_hole_d_vertical,
                    h=disk_h + 8,
                    center=true
                );

            translate([0, -p, 0])
                cylinder(
                    d=mg_horn_hole_d_vertical,
                    h=disk_h + 8,
                    center=true
                );
        }

        screw_circle_z(
            pan_screw_circle/2,
            outer_screw_d,
            disk_h + 8
        );

        for (a=[45,135,225,315])
            rotate([0,0,a])
                translate([pan_screw_circle/2, 0, disk_h/2 + 0.05])
                    cylinder(
                        d=neck_counterbore_d,
                        h=neck_counterbore_depth,
                        center=true
                    );
    }
}

module tilt_yoke_base_v1() {

    pivot_z = yoke_base_h + yoke_axis_z;

    mg_body_len = 40.0;
    mg_body_w   = 20.0;
    mg_body_h   = 40.0;

    mg_clear_len = mg_body_len + 3.0;
    mg_clear_w   = mg_body_w   + 2.8;
    mg_clear_h   = mg_body_h   + 3.2;

    mg_holder_len = 48.0;

    mg_boss_center_from_end = 17.5;

    mg_boss_y_offset = -mg_body_len/2 + mg_boss_center_from_end;

    mg_body_center_y = -mg_boss_y_offset;

    mg_boss_outset = 2.5;
    mg_body_center_x = left_arm_x - (mg_clear_h/2 + mg_boss_outset);

    mg_boss_clear_d   = 19.5;
    mg_spline_clear_d = 10.8;

    base_x_local = 56.0;
    base_y_local = 36.0;
    base_h_local = 4.0;

    arm_t_local = 4.8;
    depth_local = 32.0;

    boss_d_local = 15.5;

    shelf_t = 4.2;

    shelf_x_extra = 6.0;
    shelf_y_extra = 4.0;

    shelf_x_local = mg_clear_h + shelf_x_extra;

    servo_floor_top_z =
        pivot_z - mg_clear_w/2;

    servo_floor_center_z =
        servo_floor_top_z - shelf_t/2;

    rear_stop_t = 2.6;
    rear_stop_h = 6.5;

    difference() {
        union() {
            rounded_box(
                [base_x_local, base_y_local, base_h_local],
                r=4.5,
                center=true
            );

            translate([
                left_arm_x,
                0,
                base_h_local/2 + yoke_arm_h/2 - 1
            ])
                rounded_box(
                    [arm_t_local, depth_local, yoke_arm_h],
                    r=2.4,
                    center=true
                );

            translate([
                right_arm_x,
                0,
                base_h_local/2 + yoke_arm_h/2 - 1
            ])
                rounded_box(
                    [arm_t_local, depth_local, yoke_arm_h],
                    r=2.4,
                    center=true
                );

            translate([left_arm_x, 0, pivot_z])
                cyl_x(boss_d_local, arm_t_local + 2.4);

            translate([right_arm_x, 0, pivot_z])
                cyl_x(boss_d_local, arm_t_local + 2.4);

            translate([
                mg_body_center_x,
                mg_body_center_y,
                servo_floor_center_z
            ])
                rounded_box(
                    [
                        shelf_x_local,
                        mg_holder_len,
                        shelf_t
                    ],
                    r=1.2,
                    center=true
                );

            for (sx=[-1,1])
                hull() {
                    translate([
                        sx*(yoke_inner_w/2 + arm_t_local/2),
                        depth_local/2 - 2.0,
                        base_h_local/2
                    ])
                        cube(
                            [arm_t_local, 2.2, 2.4],
                            center=true
                        );

                    translate([
                        sx*(yoke_inner_w/2 + arm_t_local/2),
                        depth_local/2 - 2.0,
                        pivot_z - 14
                    ])
                        cube(
                            [arm_t_local, 2.2, 2.4],
                            center=true
                        );
                }
        }

        screw_circle_z(
            pan_screw_circle/2,
            m25_clear,
            base_h_local + 8
        );

        cylinder(
            d=9.0,
            h=base_h_local + 8,
            center=true
        );

        translate([left_arm_x, 0, pivot_z])
            cyl_x(
                mg_boss_clear_d,
                arm_t_local + 14
            );

        translate([left_arm_x, 0, pivot_z])
            cyl_x(
                mg_spline_clear_d,
                arm_t_local + 18
            );

        translate([
            mg_body_center_x,
            mg_body_center_y,
            pivot_z
        ])
            cube(
                [
                    mg_clear_h,
                    mg_clear_len,
                    mg_clear_w
                ],
                center=true
            );

        translate([right_arm_x,0,pivot_z])
            cyl_x(m3_clear, arm_t_local + 10);

        translate([
            right_arm_x - arm_t_local/2 - 0.35,
            0,
            pivot_z
        ])
            cyl_x(7.6,3.0);

        for (sx=[-1,1])
            translate([
                sx*(yoke_inner_w/2 + arm_t_local/2),
                0,
                pivot_z - 17
            ])
                rounded_panel_x(
                    depth_local - 9,
                    9,
                    arm_t_local + 3,
                    2.5
                );

        translate([
            mg_body_center_x + 5.0,
            depth_local/2,
            pivot_z - 17
        ])
            rounded_panel_y(8, 16, 8, 2.2);
    }
}

module camera_cradle_v1() {

    front_face_t = 2.0;

    rail_depth = max(cradle_t, cam_clear_t + 2.0);
    rail_y = 4.0;

    rail_w = 2.8;
    rail_h = cam_clear_h + 1.0;

    left_rail_x  = -(cam_clear_w/2 + rail_w/2);
    right_rail_x =  (cam_clear_w/2 + rail_w/2);

    bottom_tray_w = cam_clear_w + 7.0;
    bottom_tray_h = 5.2;
    bottom_tray_z = -cam_clear_h/2 - bottom_tray_h/2;

    wire_slot_w = 11.0;

    back_edge_y = rail_y + rail_depth/2;

    wire_slot_len = rail_depth/2 + 2.0;

    wire_slot_y_pos = back_edge_y - wire_slot_len/2 + 0.5;

    clamp_screw_x = 15;
    clamp_screw_z = 20;

    tilt_center_clear_d = 7.5;
    horn_hole_d = 2.0;
    horn_hole_z_positions = [-10, -7, -4, 4, 7, 10];
    horn_hole_y_positions = [-6, 6];

    pivot_x = right_rail_x;
    pivot_hole_d = m3_clear;

    difference() {
        union() {
            for (sx=[-1,1])
                translate([
                    sx*(cam_clear_w/2 + rail_w/2),
                    rail_y,
                    0
                ])
                    rounded_box(
                        [rail_w, rail_depth, rail_h],
                        r=1.0,
                        center=true
                    );

            translate([0, rail_y, bottom_tray_z])
                rounded_box(
                    [bottom_tray_w, rail_depth, bottom_tray_h],
                    r=1.2,
                    center=true
                );

            for (sx=[-1,1])
                translate([
                    sx*(bottom_tray_w/2 - 5.0),
                    -front_face_t/2 + 0.35,
                    -cam_clear_h/2 + 4.0
                ])
                    rounded_box(
                        [8.0, 2.4, 6.0],
                        r=0.9,
                        center=true
                    );

            for (sx=[-1,1])
                for (sz=[-1,1])
                    translate([
                        sx*clamp_screw_x,
                        rail_y + rail_depth/2 - 1.8,
                        sz*clamp_screw_z
                    ])
                        rounded_box(
                            [5.0, 3.8, 5.0],
                            r=1.1,
                            center=true
                        );
        }

        translate([left_rail_x, 0, 0])
            cyl_x(tilt_center_clear_d, rail_w + 4);

        for (hz = horn_hole_z_positions)
            translate([left_rail_x, 0, hz])
                cyl_x(horn_hole_d, rail_w + 4);

        for (hy = horn_hole_y_positions)
            translate([left_rail_x, hy, 0])
                cyl_x(horn_hole_d, rail_w + 4);

        translate([pivot_x, 0, 0])
            cyl_x(pivot_hole_d, rail_w + 4);

        translate([0, wire_slot_y_pos, bottom_tray_z])
            rounded_box(
                [wire_slot_w, wire_slot_len, bottom_tray_h + 4],
                r=2.0,
                center=true
            );

        translate([0, rail_y, bottom_tray_z])
            cylinder(
                d=wire_slot_w,
                h=bottom_tray_h + 4,
                center=true,
                $fn=40
            );
    }
}

module servo_holder() {
    import(file = servo_holder_file);
}

module assembly_preview() {
    color([0.08,0.08,0.08]) bottom_base_plate_v1();

    translate([0,0,10])
        color([0.09,0.09,0.09]) pan_servo_open_cage_v1();

    translate([0,0,60])
        color([0.14,0.14,0.14]) pan_horn_adapter_disk_v1();

    translate([0,0,66])
        color([0.10,0.10,0.10]) tilt_yoke_base_v1();

    translate([0,0,105])
        color([0.06,0.06,0.06]) camera_cradle_v1();
}

if (PART == "assembly") {
    assembly_preview();
} else if (PART == "base_plate") {
    bottom_base_plate_v1();
} else if (PART == "pan_servo_cage") {
    pan_servo_open_cage_v1();
} else if (PART == "pan_horn_disk") {
    pan_horn_adapter_disk_v1();
} else if (PART == "tilt_yoke_base") {
    tilt_yoke_base_v1();
} else if (PART == "camera_cradle") {
    camera_cradle_v1();
} else if (PART == "servo_holder") {
    servo_holder();
}
