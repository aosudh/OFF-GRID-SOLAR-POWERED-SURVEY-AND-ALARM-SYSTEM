package com.example.demo.mapper;

import com.example.demo.controller.Shuju;
import org.apache.ibatis.annotations.Mapper;

@Mapper
public interface TestMapper {


    void insert(Shuju shuju);
}
