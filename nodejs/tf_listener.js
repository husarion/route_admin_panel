var math3d = require('math3d');
const rosnodejs = require('rosnodejs');
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;

class TfListener {
    constructor() {
        this.tf_tree;
    }
}

class TfTransform {
    constructor(parent_frame, child_frame, timestamp, transform) {
        this.parent = parent_frame;
        this.child = child_frame;
        this.stamp = timestamp;
        this.transform = transform;
    }

    toString() {
        return "[" + this.parent + "] --> [" + this.child + "]";
    }
}

class TfBranch {
    constructor(tf_tree, last_update, transform) {
        this.tree = tf_tree;
        this.last_update = last_update;
        this.transform = transform;
    }
}

class TfTree {
    constructor(frame, branches) {
        this.frame_id = frame;
        this.branches = []; // array of TfBranch
        if (branches) {
            this.branches = branches;
        }
    }

    lookup_transform(parent_frame, child_frame, layer) {
        let spaces = "    ";
        let branches_copy = this.branches;
        if (parent_frame == this.frame_id) { // beginning of searched tf
            for (var i = 0, l = branches_copy.length; i < l; i++) {
                if (branches_copy[i].tree.frame_id == child_frame) {
                    let transform_stamped = branches_copy[i].transform;
                    return transform_stamped;
                } else {
                    let branch_tf = branches_copy[i].tree.lookup_transform(branches_copy[i].tree.frame_id, child_frame, layer + 1);
                    if (branch_tf) {
                        let base_tf = branches_copy[i].transform;
                        let transform_stamped = new geometry_msgs.Transform();
                        var t_end = new math3d.Transform();
                        t_end.translate(new math3d.Vector3(base_tf.translation.x, base_tf.translation.y, base_tf.translation.z));
                        let q_base = new math3d.Quaternion(base_tf.rotation.x, base_tf.rotation.y, base_tf.rotation.z, base_tf.rotation.w);
                        t_end.rotate(q_base.eulerAngles.x, q_base.eulerAngles.y, q_base.eulerAngles.z);
                        t_end.translate(new math3d.Vector3(branch_tf.translation.x, branch_tf.translation.y, branch_tf.translation.z));
                        let q_branch = new math3d.Quaternion(branch_tf.rotation.x, branch_tf.rotation.y, branch_tf.rotation.z, branch_tf.rotation.w);
                        t_end.rotate(q_branch.eulerAngles.x, q_branch.eulerAngles.y, q_branch.eulerAngles.z);
                        transform_stamped.translation.x = t_end.position.values[0];
                        transform_stamped.translation.y = t_end.position.values[1];
                        transform_stamped.translation.z = t_end.position.values[2];
                        transform_stamped.rotation.x = t_end.rotation.x;
                        transform_stamped.rotation.y = t_end.rotation.y;
                        transform_stamped.rotation.z = t_end.rotation.z;
                        transform_stamped.rotation.w = t_end.rotation.w;
                        return transform_stamped;
                    } else {
                    }
                }
            }
        } else {
            let child_transform;
            for (var i = 0, l = branches_copy.length; i < l; i++) {
                child_transform = branches_copy[i].tree.lookup_transform(parent_frame, child_frame, layer + 1);
                if (child_transform) {
                    return child_transform;
                }
            }
        }
    }

    add_transform(transform, branch_level) {
        // check if transform is parent
        if (transform.child == this.frame_id) {
            console.log("Layer: " + branch_level + ", received transform " + transform.toString() + " is parent, skip further search");
            let current_tree = new TfTree(this.frame_id, this.branches);
            let tf_branch = new TfBranch(current_tree, transform.stamp, transform.transform);
            this.branches = [];
            this.branches.push(tf_branch);
            this.frame_id = transform.parent;
        } else if (transform.parent == this.frame_id) {
            // console.log("Layer: " + branch_level + ", received transform " + transform.toString() + " is on the same level");
            let child_tree = new TfTree(transform.child);
            let tf_branch = new TfBranch(child_tree, transform.stamp, transform.transform);
            this.add_branch(tf_branch);
        } else if (this.branches.length > 0) {
            // console.log("Layer: " + branch_level + ", Check if transform " + transform.toString() + " is mathcing with barnches");
            this.branches.forEach(tf_branch => {
                tf_branch.tree.add_transform(transform, branch_level + 1);
            })
        } else {
            // console.log("Layer: " + branch_level + ", No connection between " + transform.toString() + " and current tree");
        }
    }

    add_branch(child_branch) {
        let branchUpdated = false;
        if (this.branches.length > 0) {
            this.branches.forEach(branch => {
                if (branch.tree.frame_id == child_branch.tree.frame_id) {
                    // console.log("    Found existing transform, update values");
                    branch.transform = child_branch.transform;
                    branch.last_update = child_branch.last_update;
                    branchUpdated = true;
                }
            });
        }
        if (branchUpdated == false) {
            // console.log("    Push new branch");
            this.branches.push(child_branch);
        }
        return branchUpdated;
    }

    toString(tabs) {
        let child_frames = "";
        let spaces = "    ";
        this.branches.forEach(b => {
            child_frames += spaces.repeat(tabs + 1) + b.tree.toString(tabs + 1); // + "\n"; // + " " + tf_string
        });
        return this.frame_id + "\n" + child_frames;
    }
}


module.exports = {
    TfTransform: TfTransform,
    TfListener: TfListener,
    TfTree: TfTree,
    TfBranch: TfBranch
}