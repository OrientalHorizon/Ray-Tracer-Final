use crate::aabb::Aabb;
use crate::material::Material;
use crate::ray::Ray;
use crate::rt_weekend::{degrees_to_radians, INFINITY};
use crate::vec3::Point3;
use crate::vec3::{dot, Vec3};
use std::sync::Arc;

#[derive(Clone, Default)]
pub struct HitRecord {
    pub p: Point3,
    pub normal: Vec3,
    pub mat_ptr: Option<Arc<dyn Material>>,
    pub t: f64,
    pub u: f64,
    pub v: f64,
    pub front_face: bool,
}

impl HitRecord {
    pub fn new() -> Self {
        Self {
            p: Point3::new(),
            normal: Vec3::new(),
            mat_ptr: None,
            t: 0.0,
            u: 0.0,
            v: 0.0,
            front_face: false,
        }
    }
    // pub fn construct(p: &Point3, normal: &Vec3, t: f64, front_face: bool) -> Self {
    //     Self {
    //         p: *p,
    //         normal: *normal,
    //         t,
    //         front_face,
    //     }
    // }
    pub fn set_face_normal(&mut self, r: &Ray, outward_normal: &Vec3) {
        self.front_face = dot(&r.direction(), outward_normal) < 0.0;
        self.normal = if self.front_face {
            *outward_normal
        } else {
            -*outward_normal
        };
    }
}

pub trait Hittable: Send + Sync {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64, rec: &mut HitRecord) -> bool;
    fn bounding_box(&self, time0: f64, time1: f64, output_box: &mut Aabb) -> bool;
}

pub struct Translate {
    pub ptr: Arc<dyn Hittable>,
    pub offset: Vec3,
}
impl Translate {
    pub fn construct(p: Arc<dyn Hittable>, displacement: &Vec3) -> Self {
        Self {
            ptr: Arc::clone(&p),
            offset: *displacement,
        }
    }
}
impl Hittable for Translate {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64, rec: &mut HitRecord) -> bool {
        let moved_r = Ray::construct(&(r.origin() - self.offset), &r.direction(), r.time());
        if !self.ptr.hit(&moved_r, t_min, t_max, rec) {
            return false;
        }
        rec.p += self.offset;
        true
    }
    fn bounding_box(&self, _time0: f64, _time1: f64, output_box: &mut Aabb) -> bool {
        if !self.ptr.bounding_box(_time0, _time1, output_box) {
            return false;
        }
        *output_box = Aabb::construct(
            &(output_box.minimum() + self.offset),
            &(output_box.maximum() + self.offset),
        );
        println!("bbox: {:?}", *output_box);
        true
    }
}

pub struct RotateX {
    pub ptr: Arc<dyn Hittable>,
    pub sin_theta: f64,
    pub cos_theta: f64,
    pub hasbox: bool,
    pub bbox: Aabb,
}
impl RotateX {
    pub fn construct(p: Arc<dyn Hittable>, angle: f64) -> Self {
        let radians = angle.to_radians();
        let sin_theta = radians.sin();
        let cos_theta = radians.cos();
        let mut bbox = Aabb::new();
        let hasbox = p.bounding_box(0.0, 1.0, &mut bbox);
        println!("bbox: {:?}", bbox);
        let mut mini = Point3::construct(&[INFINITY, INFINITY, INFINITY]);
        let mut maxi = Point3::construct(&[-INFINITY, -INFINITY, -INFINITY]);

        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let x = i as f64 * bbox.maximum().x() + (1.0 - i as f64) * bbox.minimum().x();
                    let y = j as f64 * bbox.maximum().y() + (1.0 - j as f64) * bbox.minimum().y();
                    let z = k as f64 * bbox.maximum().z() + (1.0 - k as f64) * bbox.minimum().z();

                    let pre_rot = Vec3::construct(&[x, y, z]);
                    let tester = rotate_vec_x(&pre_rot, -sin_theta, cos_theta);
                    for c in 0..3 {
                        mini.e[c] = tester.e[c].min(mini.e[c]);
                        maxi.e[c] = tester.e[c].max(maxi.e[c]);
                    }
                }
            }
        }
        bbox = Aabb::construct(&mini, &maxi);
        Self {
            ptr: Arc::clone(&p),
            sin_theta,
            cos_theta,
            hasbox,
            bbox,
        }
    }
}
impl Hittable for RotateX {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64, rec: &mut HitRecord) -> bool {
        let origin = rotate_vec_x(&r.origin(), self.sin_theta, self.cos_theta);
        let direction = rotate_vec_x(&r.direction(), self.sin_theta, self.cos_theta);
        let rotated_r = Ray::construct(&origin, &direction, r.time());

        if !self.ptr.hit(&rotated_r, t_min, t_max, rec) {
            return false;
        }

        let p = rotate_vec_x(&rec.p, -self.sin_theta, self.cos_theta);
        let normal = rotate_vec_x(&rec.normal, -self.sin_theta, self.cos_theta);
        rec.p = p;
        rec.normal = normal;

        true
    }
    fn bounding_box(&self, _time0: f64, _time1: f64, output_box: &mut Aabb) -> bool {
        *output_box = self.bbox;
        self.hasbox
    }
}

pub struct RotateY {
    pub ptr: Arc<dyn Hittable>,
    pub sin_theta: f64,
    pub cos_theta: f64,
    pub hasbox: bool,
    pub bbox: Aabb,
}
impl RotateY {
    pub fn construct(p: Arc<dyn Hittable>, angle: f64) -> Self {
        let radians: f64 = degrees_to_radians(angle);
        let sin_theta = radians.sin();
        let cos_theta = radians.cos();
        let mut bbox = Aabb::new();
        let hasbox = p.bounding_box(0.0, 1.0, &mut bbox);
        println!("bbox: {:?}", bbox);
        let mut mini = Point3::construct(&[INFINITY, INFINITY, INFINITY]);
        let mut maxi = Point3::construct(&[-INFINITY, -INFINITY, -INFINITY]);

        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let x = i as f64 * bbox.maximum().x() + (1.0 - i as f64) * bbox.minimum().x();
                    let y = j as f64 * bbox.maximum().y() + (1.0 - j as f64) * bbox.minimum().y();
                    let z = k as f64 * bbox.maximum().z() + (1.0 - k as f64) * bbox.minimum().z();

                    let pre_rotation = Vec3::construct(&[x, y, z]);

                    let tester = rotate_vec_y(&pre_rotation, -sin_theta, cos_theta);

                    for c in 0..3 {
                        mini.e[c] = mini.e[c].min(tester.e[c]);
                        maxi.e[c] = maxi.e[c].max(tester.e[c]);
                    }
                }
            }
        }
        bbox = Aabb::construct(&mini, &maxi);
        Self {
            ptr: Arc::clone(&p),
            sin_theta,
            cos_theta,
            hasbox,
            bbox,
        }
    }
}
impl Hittable for RotateY {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64, rec: &mut HitRecord) -> bool {
        let origin = rotate_vec_y(&r.origin(), self.sin_theta, self.cos_theta);
        let direction = rotate_vec_y(&r.direction(), self.sin_theta, self.cos_theta);

        let rotated_r = Ray::construct(&origin, &direction, r.time());

        if !self.ptr.hit(&rotated_r, t_min, t_max, rec) {
            return false;
        }

        let p = rotate_vec_y(&rec.p, -self.sin_theta, self.cos_theta);
        let normal = rotate_vec_y(&rec.normal, -self.sin_theta, self.cos_theta);
        rec.p = p;
        rec.normal = normal;

        true
    }
    fn bounding_box(&self, _time0: f64, _time1: f64, output_box: &mut Aabb) -> bool {
        *output_box = self.bbox;
        self.hasbox
    }
}
pub struct RotateZ {
    pub ptr: Arc<dyn Hittable>,
    pub sin_theta: f64,
    pub cos_theta: f64,
    pub hasbox: bool,
    pub bbox: Aabb,
}
impl RotateZ {
    pub fn construct(p: Arc<dyn Hittable>, angle: f64) -> Self {
        let radians = angle.to_radians();
        let sin_theta = radians.sin();
        let cos_theta = radians.cos();
        let mut bbox = Aabb::new();
        let hasbox = p.bounding_box(0.0, 1.0, &mut bbox);
        println!("bbox: {:?}", bbox);
        let mut mini = Point3::construct(&[INFINITY, INFINITY, INFINITY]);
        let mut maxi = Point3::construct(&[-INFINITY, -INFINITY, -INFINITY]);

        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let x = i as f64 * bbox.maximum().x() + (1.0 - i as f64) * bbox.minimum().x();
                    let y = j as f64 * bbox.maximum().y() + (1.0 - j as f64) * bbox.minimum().y();
                    let z = k as f64 * bbox.maximum().z() + (1.0 - k as f64) * bbox.minimum().z();

                    let pre_rotate = Vec3::construct(&[x, y, z]);
                    let tester = rotate_vec_z(&pre_rotate, -sin_theta, cos_theta);
                    for c in 0..3 {
                        mini.e[c] = tester.e[c].min(mini.e[c]);
                        maxi.e[c] = tester.e[c].max(maxi.e[c]);
                    }
                }
            }
        }
        bbox = Aabb::construct(&mini, &maxi);
        Self {
            ptr: Arc::clone(&p),
            sin_theta,
            cos_theta,
            hasbox,
            bbox,
        }
    }
}
impl Hittable for RotateZ {
    fn hit(&self, r: &Ray, t_min: f64, t_max: f64, rec: &mut HitRecord) -> bool {
        let origin = rotate_vec_z(&r.origin(), self.sin_theta, self.cos_theta);
        let direction = rotate_vec_z(&r.direction(), self.sin_theta, self.cos_theta);

        let rotated_r = Ray::construct(&origin, &direction, r.time());

        if !self.ptr.hit(&rotated_r, t_min, t_max, rec) {
            return false;
        }

        let p = rotate_vec_z(&rec.p, -self.sin_theta, self.cos_theta);
        let normal = rotate_vec_z(&rec.normal, -self.sin_theta, self.cos_theta);
        rec.p = p;
        rec.normal = normal;

        true
    }
    fn bounding_box(&self, _time0: f64, _time1: f64, output_box: &mut Aabb) -> bool {
        *output_box = self.bbox;
        self.hasbox
    }
}

fn rotate_vec_y(v: &Vec3, sin_theta: f64, cos_theta: f64) -> Vec3 {
    Vec3::construct(&[
        cos_theta * v.x() - sin_theta * v.z(),
        v.y(),
        sin_theta * v.x() + cos_theta * v.z(),
    ])
}
fn rotate_vec_x(v: &Vec3, sin_theta: f64, cos_theta: f64) -> Vec3 {
    Vec3::construct(&[
        v.x(),
        cos_theta * v.y() + sin_theta * v.z(),
        -sin_theta * v.y() + cos_theta * v.z(),
    ])
}
fn rotate_vec_z(v: &Vec3, sin_theta: f64, cos_theta: f64) -> Vec3 {
    Vec3::construct(&[
        cos_theta * v.x() + sin_theta * v.y(),
        -sin_theta * v.x() + cos_theta * v.y(),
        v.z(),
    ])
}
