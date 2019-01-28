---
layout: archive
author_profile: false
permalink: /publications/
---

{% include base_path %}
<!-- <h3 class="archive__subtitle">Examples from various domains</h3>		This destroys lists -->
{% assign items = site.posts | sort: 'title' %}

{% include paginator.html %}
