"""
    Date: 2019/08/15
"""
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys

def answer_question(output, ans_count):
    if "what" in output:
        if "team" in output or "name" in output:
            ans = "the name of our team is team kamerider"
            ans_count += 3
            return ans
        
        if "day" in output or "today" in output:
            ans = "today is 16 agust year 2019"
            ans_count += 3
            return ans

        if "capital" in output or "japan" in output:
            ans = "the capital of japan is tokyo"
            ans_count += 3
            return ans
        
        if "height" in output or "mountain" in output:
            ans = "the height of mountain fuji is about 3776 meters"
            ans_count += 3
            return ans
        
        if "longest" in output or "river" in output:
            ans = "the longest river is nile river"
            ans_count += 3
            return ans
        
        if "heaviest" in output or "animal" in output:
            ans = "the heaviest animal is blue whale"
            ans_count += 3
            return ans

    elif "where" in output:
        if "from" in output or "you" in output:
            ans = "i come from nankai university tianjin city china"
            ans_count += 3
            return ans
        
        if "we" in output or "are" in output:
            ans = "we are in the nagaoka"
            ans_count += 3
            return ans
        
    elif "who" in output or "american" in output:
        ans = "the american president is Donald Trump"
        ans_count += 3
        return ans
    
    elif "how" in output or "many" in output:
        ans = "the cows have four legs"
        ans_count += 3
        return ans

    else:
        ans = "none match"
        ans_count += 1
        return ans
    
    if ans_count >= 9:
        ans = "end"
        return ans
