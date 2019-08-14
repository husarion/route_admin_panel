class TargetList {
    constructor() {
        this.targets = [];
    }

    add_target(target) {
        this.targets.push(target);
    }

    get_target_by_id(target_id) {
        console.log("Searching for target:", target_id);
        for (var i = 0; i < this.targets.length; i++) {
            if (this.targets[i].id === target_id) {
                return this.targets[i];
            }
        }
    }

    remove_target(target_id) {
        console.log("Removing target:", target_id);
        for (var i = 0; i < this.targets.length; i++) {
            if (this.targets[i].id === target_id) {
                this.targets.splice(i, 1);
            }
        }
    }

    get_next_id() {
        let highest_id = 0;
        for (var i = 0; i < this.targets.length; i++) {
            if (this.targets[i].id >= highest_id) {
                highest_id = this.targets[i].id;
            }
        }
        return highest_id + 1;
    }
}

class Target {
    constructor(id, x, y, theta, label) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.label = label;
    }
}

module.exports = {
    TargetList: TargetList,
    Target: Target
}