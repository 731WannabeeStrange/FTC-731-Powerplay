package org.firstinspires.ftc.teamcode.utils;

import java.util.Stack;

public class SizedStack<T> extends Stack<T> {
    private int maxSize;

    public SizedStack(int maxSize) {
        super();
        this.maxSize = maxSize;
    }

    @Override
    public T push(T object) {
        while (this.size() >= maxSize) {
            this.remove(0);
        }
        return super.push(object);
    }
}
