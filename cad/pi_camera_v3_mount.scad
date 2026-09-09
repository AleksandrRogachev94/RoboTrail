// Simple mount for custom Camera Module / Sensor
// Updated: Swapped horizontal and vertical camera hole orientation

$fn = 64;

// ---- Target lens center height ----
target_lens_height = 70;

// ---- Camera board mounting pattern (FLIPPED) ----
cam_hole_x   = 20.5;  // horizontal hole spacing
cam_hole_y   = 12.2;  // vertical hole spacing
cam_hole_d   = 2.2;   // M2 clearance
boss_d       = 6;     // camera standoff boss diameter
boss_h       = 3.5;   // standoff height to clear rear components

// ---- Vertical camera face ----
face_margin  = 6;
face_w       = 33;    // matches base width
face_h       = cam_hole_y + 2 * face_margin;   // 24.2mm
face_t       = 3;

// ---- Vertical riser (load-bearing column) ----
riser_w   = face_w;  // 33mm
riser_t   = 5;       // 5mm thickness
overlap   = 0.5;     // overlap for solid geometry merging

// ---- Horizontal mounting flange ----
flange_depth      = 12;   // 12mm depth
flange_t          = 3;
flange_hole_d     = 2.2;  // M2 clearance
flange_hole_inset = 8;    // inset from left/right edges

// ---- Base gussets ----
gusset_t     = 3;

// Calculate riser height:
// target_lens_height = flange_t + riser_h + face_h/2
riser_h      = target_lens_height - flange_t - face_h / 2; // 54.9mm
gusset_reach = min(riser_h, 20); // 20mm vertical leg reach

assert(riser_h > 5, "target_lens_height too small for face_h/flange_t combination");

module camera_face() {
    difference() {
        translate([-face_w/2, 0, 0])
            cube([face_w, face_t, face_h]);

        // Camera mounting holes through face and bosses
        for (dx = [-cam_hole_x/2, cam_hole_x/2])
            for (dz = [-cam_hole_y/2, cam_hole_y/2])
                translate([dx, -1, face_h/2 + dz])
                    rotate([-90, 0, 0])
                        cylinder(d = cam_hole_d, h = face_t + boss_h + 2);
    }

    // Standoff bosses on front face
    for (dx = [-cam_hole_x/2, cam_hole_x/2])
        for (dz = [-cam_hole_y/2, cam_hole_y/2])
            translate([dx, face_t, face_h/2 + dz])
                rotate([-90, 0, 0])
                    difference() {
                        cylinder(d = boss_d, h = boss_h);
                        cylinder(d = cam_hole_d, h = boss_h + 1);
                    }
}

module riser() {
    translate([-riser_w/2, 0, 0])
        cube([riser_w, riser_t, flange_t + riser_h + overlap]);
}

module mount_flange() {
    translate([-face_w/2, 0, 0])
        difference() {
            cube([face_w, flange_depth, flange_t]);

            // Center holes in the exposed flange area in front of the 5mm riser
            for (dx = [flange_hole_inset, face_w - flange_hole_inset])
                translate([dx, (flange_depth - riser_t) / 2, -1])
                    cylinder(d = flange_hole_d, h = flange_t + 2);
        }
}

module gusset(x_pos) {
    hull() {
        translate([x_pos, 0, 0]) cube([gusset_t, 0.01, 0.01]);
        translate([x_pos, flange_depth, 0]) cube([gusset_t, 0.01, 0.01]);
        translate([x_pos, 0, gusset_reach]) cube([gusset_t, 0.01, 0.01]);
    }
}

module camera_mount() {
    mount_flange();
    riser();
    translate([0, 0, flange_t + riser_h])
        camera_face();

    gusset(-riser_w/2);
    gusset(riser_w/2 - gusset_t);
}

camera_mount();
