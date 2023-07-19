package com.example.demo.controller;

import com.example.demo.mapper.TestMapper;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.ResponseBody;

@Controller
public class test {

    @Autowired
    TestMapper testMapper;


    @ResponseBody
    @RequestMapping("/Increase")
    public void Increase(String data){
        Shuju shuju=new Shuju();
        shuju.setN1(data.substring(0, 9));
        shuju.setN2(data.substring(9, 18));
        shuju.setN3(data.substring(18, 19));
        shuju.setN4(data.substring(19, 21));
        shuju.setN5(data.substring(21, 25));
        shuju.setN6(data.substring(25, 29));
        shuju.setN7(data.substring(29, 34));
        testMapper.insert(shuju);
    }

    public static void main(String[] args) {

        String data="N39.75159E13.921941125159111V11.7W";
//        String data="N39.75159E13.921941125159111V11.6W";
        System.out.println(data.substring(0, 9));
        System.out.println(data.substring(9, 18));
        System.out.println(data.substring(18, 19));
        System.out.println(data.substring(19, 21));
        System.out.println(data.substring(21, 25));
        System.out.println(data.substring(25, 29));
        System.out.println(data.substring(29, 34));

    }

}
