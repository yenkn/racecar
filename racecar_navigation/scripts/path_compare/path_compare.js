#!/usr/bin/env node
'use strict';
/**
 * This example demonstrates simple receiving of messages over the ROS system.
 */

// Require rosnodejs itself
    const process = require('process');
const rosnodejs = require('rosnodejs');
const { shapeSimilarity } = require('curve-matcher');
// Requires the std_msgs message package
const std_msgs = rosnodejs.require('std_msgs').msg;
const GetPathSimilarity = rosnodejs.require('racecar_msgs').srv.GetPathSimilarity;

function listener() {
    // Register node with ROS master
    rosnodejs.initNode('/path_compare', { onTheFly: true })
        .then((rosNode) => {
            const comparePath = (req, resp) => {
                const path1 = req.path1.poses.map(pose => ({ x: pose.pose.position.x, y: pose.pose.position.y }));
                const path2 = req.path2.poses.map(pose => ({ x: pose.pose.position.x, y: pose.pose.position.y }));
                try {
                    resp.similarity = shapeSimilarity(path1, path2);
                } catch (e) {
                    return false;
                }
                return true;
            };

            let service = rosNode.advertiseService('/compare_path', GetPathSimilarity, comparePath);
        });
}

if (require.main === module) {
    // Invoke Main Listener Function
    listener();
}
