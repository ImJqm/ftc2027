package org.firstinspires.ftc.teamcode.pedro;

import java.util.List;

public class Sequencer {

    private int ticker = 0;
    private int[] intervals;
    private List<Runnable> tasks;

    private boolean engaged = false;
    String status = "";

    // FOLLOWING FORMAT
    // [t1, t2]
    // [a1, a2, a3]
    // a1 happanes whole t < t1
    // a2 happens while t < t2
    // a3 is the stop action
    public Sequencer() {
        intervals = null;
        tasks = null;
    }
    public Sequencer(int[] intervals, List<Runnable> tasks) {
        this.intervals = intervals;
        this.tasks = tasks;
    }

    public void setIntervals(int[] intervals) {
        this.intervals = intervals;
    }

    public void setTasks(List<Runnable> tasks) {
        this.tasks = tasks;
    }

    public void update() {

        if (engaged) {
            ticker++;

            for (int i = 0; i < intervals.length; i++) {
                if (ticker < intervals[i]) {
                    if (i == 0) {
                        status = "First Time";
                    } else {
                        status = "Running";
                    }
                    tasks.get(i).run();
                    break;
                }
            }

            if (ticker >= intervals[intervals.length-1]) {
                engaged = false;
                tasks.get(intervals.length-1).run();
            }
        }

    }

    public void engage() {
        this.engaged = true;
    }

    public boolean isEngaged() {
        return this.engaged;
    }



}
