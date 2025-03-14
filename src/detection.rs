//! Tag detection types.

use crate::{
    matd::MatdRef,
    pose::{Pose, PoseEstimation, TagParams},
};
use apriltag_sys as sys;
use std::{
    ffi::c_int,
    fmt::{self, Debug, Formatter},
    mem::{ManuallyDrop, MaybeUninit},
    ptr::{null_mut, NonNull},
};

/// Represent a marker detection outcome.
#[repr(transparent)]
pub struct Detection {
    ptr: NonNull<sys::apriltag_detection_t>,
}

impl Detection {
    /// Get the marker ID.
    pub fn id(&self) -> usize {
        unsafe { self.ptr.as_ref().id as usize }
    }

    /// Get the Hamming distance to the target tag.
    pub fn hamming(&self) -> usize {
        unsafe { self.ptr.as_ref().hamming as usize }
    }

    /// Indicate the _goodness_ of the detection.
    pub fn decision_margin(&self) -> f32 {
        unsafe { self.ptr.as_ref().decision_margin }
    }

    /// Get the center coordinates in form of `[x, y]`.
    pub fn center(&self) -> [f64; 2] {
        unsafe { self.ptr.as_ref().c }
    }

    /// Get the corner coordinates in form of `[[x, y]; 4]`.
    pub fn corners(&self) -> [[f64; 2]; 4] {
        unsafe { self.ptr.as_ref().p }
    }

    /// Get the homography matrix.
    pub fn homography(&self) -> MatdRef<'_> {
        unsafe { MatdRef::from_ptr(self.ptr.as_ref().H) }
    }

    /// Estimates the pose of tag with specified number of iterations.
    pub fn estimate_tag_pose_orthogonal_iteration(
        &self,
        params: &TagParams,
        n_iters: usize,
    ) -> Vec<PoseEstimation> {
        let mut info = sys::apriltag_detection_info_t {
            det: self.ptr.as_ptr(),
            tagsize: params.tagsize,
            fx: params.fx,
            fy: params.fy,
            cx: params.cx,
            cy: params.cy,
        };

        let poses: Vec<_> = unsafe {
            let pose1: *mut sys::apriltag_pose_t = null_mut();
            let mut err1: f64 = 0.0;
            let pose2: *mut sys::apriltag_pose_t = null_mut();
            let mut err2: f64 = 0.0;

            sys::estimate_tag_pose_orthogonal_iteration(
                &mut info as *mut _,
                &mut err1,
                pose1,
                &mut err2,
                pose2,
                n_iters as c_int,
            );

            let pose1 = if !(*pose1).R.is_null() {
                Some(PoseEstimation {
                    pose: Pose(*pose1),
                    error: err1,
                })
            } else {
                None
            };

            let pose2 = if !(*pose2).R.is_null() {
                Some(PoseEstimation {
                    pose: Pose(*pose2),
                    error: err2,
                })
            } else {
                None
            };

            pose1.into_iter().chain(pose2).collect()
        };

        poses
    }

    /// Estimates the pose of tag.
    pub fn estimate_tag_pose(&self, params: &TagParams) -> Option<Pose> {
        let mut info = sys::apriltag_detection_info_t {
            det: self.ptr.as_ptr(),
            tagsize: params.tagsize,
            fx: params.fx,
            fy: params.fy,
            cx: params.cx,
            cy: params.cy,
        };

        unsafe {
            let mut pose: MaybeUninit<sys::apriltag_pose_t> = MaybeUninit::uninit();
            sys::estimate_tag_pose(&mut info as *mut _, pose.as_mut_ptr());
            let pose = pose.assume_init();

            (!pose.R.is_null()).then(|| Pose(pose))
        }
    }

    /// Creates an instance from pointer.
    ///
    /// The pointer will be managed by the type. Do not run manual deallocation on the pointer.
    /// Panics if the pointer is null.
    ///
    /// # Safety
    /// The method is safe when the pointer was created by [apriltag_detector_detect](sys::apriltag_detector_detect).
    pub unsafe fn from_raw(ptr: *mut sys::apriltag_detection_t) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }

    /// Returns the underlying pointer.
    pub fn into_raw(self) -> NonNull<sys::apriltag_detection_t> {
        ManuallyDrop::new(self).ptr
    }
}

impl Debug for Detection {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("Detection")
            .field("id", &self.id())
            .field("hamming", &self.hamming())
            .field("decision_margin", &self.decision_margin())
            .field("center", &self.center())
            .field("corners", &self.corners())
            .finish()?;
        Ok(())
    }
}

impl Drop for Detection {
    fn drop(&mut self) {
        unsafe {
            sys::apriltag_detection_destroy(self.ptr.as_ptr());
        }
    }
}
