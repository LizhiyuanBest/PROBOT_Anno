#!/usr/bin/env python
# -*- coding: utf-8 -*-


class Quaternion:
    def __init__(self,array):
        self.w=0
        self.x=array[0]
        self.y=array[1]
        self.z=array[2]
        self.array=array
    def toArray(self):
        return [self.x,self.y,self.z,self.w]
    def __add__(self,quaternion):
        result=Quaternion(self.array)
        result.w+=quaternion.w
        result.x+=quaternion.x
        result.y+=quaternion.y
        result.z+=quaternion.z
        return result
    def __sub__(self,quaternion):
        result=Quaternion(self.array)
        result.w-=quaternion.w
        result.x-=quaternion.x
        result.y-=quaternion.y
        result.z-=quaternion.z
        return result
    def multiplication(self,quaternion):
        result=Quaternion(self.array)
        result.w=self.w*quaternion.w-self.x*quaternion.x-self.y*quaternion.y-self.z*quaternion.z
        result.x=self.w*quaternion.x+self.x*quaternion.w+self.y*quaternion.z-self.z*quaternion.y
        result.y=self.w*quaternion.y-self.x*quaternion.z+self.y*quaternion.w+self.z*quaternion.x
        result.z=self.w*quaternion.z+self.x*quaternion.y-self.y*quaternion.x+self.z*quaternion.w
        return result
    def divides(self,quaternion):
        result=Quaternion(self.array)
        return result.multiplication(quaternion.inverse())
    def mod(self):
        return pow((pow(self.x,2)+pow(self.y,2)+pow(self.z,2)+pow(self.w,2)),1/2)
    def star(self):
        result=Quaternion(self.array)
        result.w=self.w
        result.x=-self.x
        result.y=-self.y
        result.z=-self.z
        return result
    def inverse(self):
        result=Quaternion(self.array)
        moder=self.mod()
        result.w/=moder
        result.x/=moder
        result.y/=moder
        result.z/=moder
        return result
    def __str__(self):
        return str(self.x)+"i "+str(self.y)+"j "+str(self.z)+"k "+str(self.w)