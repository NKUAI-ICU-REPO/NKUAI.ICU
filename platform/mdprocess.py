# -*- coding: utf-8 -*-
"""
@Project ：index.html 
@File    ：mdprocess.py
@Author  ：BreezeConfirming
@Date    ：2024/2/20 上午12:15 
@Mail    : BreezeConfirming@163.com
=====================
Edited On PyCharm
Developing Time Stamp <- 2024
=====================

@CopyRight Ming.Yan all right reserved

"""


import markdown

import os

from pathlib import Path


import lxml.html

from bs4 import BeautifulSoup as bsp

from bs4 import SoupStrainer
import json
from markdownify import markdownify as md


import argparse


parser=argparse.ArgumentParser(description="md process")

parser.add_argument("--target_dir",type=str,help="md file path",required=False)
course_json_url ="../lesson.json"


top_filepath = Path(__file__).absolute().parent.parent


target_dirs=["grade1","grade2","grade3"]

def md2html(mdfile:str)->str:
    with open(mdfile, 'r') as f:
        text = f.read()
        html = markdown.markdown(text)
        return html

def extract_head(mdpath:str)->str:

    # with open(mdpath,'r') as f:
    #     text = f.read()
    #     html = markdown.markdown(text)


    html = md2html(mdpath)

    esoup = bsp(html,'html.parser')
    h1title=esoup.find_all('h1')[0].text

    return h1title


def is_li_with_a(tag):
    return tag.name == 'li' and tag.a
if __name__ == '__main__':
    slide_html = md2html("../_sidebar.md")
    print(top_filepath)
    print(slide_html)


    print("=======")

    soup=bsp(slide_html, 'html.parser')

    ul_tags = soup.find_all('ul')


    for c in range(len(target_dirs)):
        mv_targets = []
        mv_head = []
        target_path = str(top_filepath) + "/lesson/" + target_dirs[c]
        for dirpath,dirnames,filenames in os.walk(target_path):
            for file in filenames:
                if file.endswith('.md'):
                    file_path = os.path.join(dirpath,file)

                    mv_targets.append(dirpath[len(str(top_filepath)):]+"/"+file)
                    mv_head.append(extract_head(file_path))

        ul_li = ul_tags[c+1].find_all(is_li_with_a)


        li_href = [li.a['href'] for li in ul_li]



        for idx,mv in enumerate(mv_targets):
            if mv in li_href:
                continue
            else:
                new_li = soup.new_tag('li')
                new_a = soup.new_tag('a',href=mv)
                new_a.string = mv_head[idx]
                new_li.append(new_a)


                ul_tags[c+1].append(new_li)

    rhtml = soup.prettify()

    markdown_res=md(rhtml)


    # print(markdown_res)

    with open("../_sidebar.md",'w') as f:
        f.write(markdown_res)