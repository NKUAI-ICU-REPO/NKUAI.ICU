# -*- coding: utf-8 -*-
"""
@Project ：index.html 
@File    ：lessonparse.py
@Author  ：BreezeConfirming
@Date    ：2024/2/19 下午8:58 
@Mail    : BreezeConfirming@163.com
=====================
Edited On PyCharm
Developing Time Stamp <- 2024
=====================

@CopyRight Ming.Yan all right reserved

"""


import os
from pathlib import Path

import logging

import re


from bs4 import BeautifulSoup as bs
from bs4.formatter import HTMLFormatter
from bs4 import SoupStrainer

import pandas as pd
import json


from typing import Tuple,Optional,Dict,Any

grade1, grade2, grade3 = {}, {}, {}


import markdown


from functools import wraps



grade_urls = [
    "../lesson/grade1/","../lesson/grade2/",
    "../lesson/grade3/","../lesson/grade4/"
]

course_json_url = "../lesson.json"
markdown_template="""
# {course_id} {course_name}

## 课程纪要

## 课外拓展

## Info Link


> 如果你愿意提供任何信息与观点，请在下方评论区留言，网站维护者会在第一时间看到，将评论优质内容添加到主页🖱
>
> 期待大家的共同建设🏗
"""
def walk_lesson():
    global grade1, grade2, grade3
    if not bool(grade1):
        return


    def walk_grade(url,grade:Dict)->None:
        for course_id, course_name in grade.items():
            course_url = url + course_id
            if not Path(course_url).exists():
                os.mkdir(course_url)
                with open(course_url + "/main.md", 'w', encoding='utf-8') as f:
                    f.write(markdown_template.format(course_id=course_id, course_name=course_name))

    for i,url in enumerate(grade_urls):
        if i == 0:
            walk_grade(url,grade1)
        elif i == 1:
            walk_grade(url,grade2)
        elif i == 2:
            walk_grade(url,grade3)
        else:
            pass
    # with open(course_json_url,'r',encoding='utf-8') as f:
    #     courses = json.load(f)
    #
    # assert len(courses) == len(grade_urls)
    # assert all(isinstance(x, dict) for x in courses)




if __name__ == '__main__':
    with open('./info/educultivate.html', 'r', encoding='utf-8') as f:
        html_content = f.read()

    df = pd.DataFrame()

    soup = bs(html_content, 'html.parser')

    # tags = soup.find_all('tr')
    #
    # if tag is not None:
    #     print(tag.text)

    # re_course = re.compile('course')
    #
    # re_tags = soup.find_all(class_=re_course)



    tr_tags = soup.find_all('tr')

    tr_depth = 3

    rules = []

    for idx, tr in enumerate(tr_tags):

        if tr.has_attr('colspan') or idx < tr_depth:
            continue
        td_tags = tr.find_all('td')

        if len(td_tags) < 1:
            continue
        if 'colspan' in td_tags[0].attrs:
            continue
        id_number = td_tags[0].text
        flag = re.match(r'^ENTD', id_number)
        if flag or len(id_number) != 8:
            continue
        course_info = tr.find('td', class_='course')

        course_href = course_info.a['href']
        class_name = course_info.a.text
        td_config = tr.find_all('td', class_='credit_hour')
        class_score = td_config[0].text
        class_deal = re.search(r"(\d){1}", td_config[1].text)
        if class_deal is not None:
            class_sem = class_deal.group(0)
        else:
            class_sem = re.search(r"(\d){1}", td_config[2].text).group(0)
        rules.append((class_sem, id_number, class_name, course_href,class_score))

    fri = ['1', '2']
    sec = ['3', '4']
    thr = ['5', '6']
    # four = [7,8]


    for idx in range(len(rules)):
        sem_num = rules[idx][0]
        if rules[idx][0] in fri:
            grade1[rules[idx][1]] = rules[idx][2]
        elif rules[idx][0] in sec:
            grade2[rules[idx][1]] = rules[idx][2]
        elif rules[idx][0] in thr:
            grade3[rules[idx][1]] = rules[idx][2]

    grades = [grade1, grade2, grade3]

    json_str = json.dumps(grades, ensure_ascii=False, indent=4)
    print(json_str)


    walk_lesson()


    # with open(course_json_url, 'w', encoding='utf-8') as f:
    #     json.dump(grades, f, ensure_ascii=False, indent=4)