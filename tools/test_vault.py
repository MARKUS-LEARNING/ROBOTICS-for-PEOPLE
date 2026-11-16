"""Edge cases where vault.py must agree with Obsidian. Run: python3 -m unittest discover -s tools"""
import os, sys, tempfile, textwrap, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vault


def note(body, fm='title: T\ntags:\n  - a\n'):
    with tempfile.NamedTemporaryFile('w', suffix='.md', delete=False, encoding='utf-8') as f:
        f.write(f'---\n{fm}---\n{textwrap.dedent(body)}')
    try:
        return vault.load_note(f.name)
    finally:
        os.unlink(f.name)


class Links(unittest.TestCase):
    def test_alias_heading_embed(self):
        n = note('See [[Forward_Kinematics#Example|FK]] and ![[diagram.png]].')
        self.assertEqual([(l.target, l.heading, l.alias, l.embed) for l in n.links],
                         [('Forward_Kinematics', 'Example', 'FK', False), ('diagram.png', None, None, True)])

    def test_escaped_pipe_in_table(self):
        n = note('| a | [[Piezoelectricity\\|piezoelectric]] |')
        self.assertEqual((n.links[0].target, n.links[0].alias), ('Piezoelectricity', 'piezoelectric'))

    def test_code_is_not_linked(self):
        n = note('```dataview\nLIST WHERE contains(file.outlinks, [[Robots]])\n```\nand `[[Inline]]` but [[Real]]')
        self.assertEqual([l.target for l in n.links], ['Real'])

    def test_frontmatter_links(self):
        n = note('body', fm='title: T\nrelated:\n  - "[[SLAM]]"\n')
        self.assertEqual([(l.target, l.source) for l in n.links], [('SLAM', 'frontmatter')])

    def test_markdown_links(self):
        n = note('[folder](../13_Tools/) [site](https://x.org) [n](Other.md#h) [top](#top)')
        self.assertEqual([(l.target, l.relative) for l in n.links], [('../13_Tools/', True), ('Other.md', True)])

    def test_resolution_case_and_unicode(self):
        v = vault.Vault()
        real = v.notes[0]
        upper = vault.Link(target=real.name.upper(), heading=None, alias=None, embed=False, source='body')
        self.assertIs(v.resolve(upper, real), real)
        missing = vault.Link(target='No_Such_Note_xyz', heading=None, alias=None, embed=False, source='body')
        self.assertIsNone(v.resolve(missing, real))


class FixLinks(unittest.TestCase):
    def test_rewrites_outside_code_and_keeps_display_text(self):
        fix = {'Machine Learning': 'Machine_Learning', 'Reinforcement Learning (RL)': 'Reinforcement_Learning'}.get
        text = ('See [[Machine Learning]] and [[Reinforcement Learning (RL)|RL]].\n'
                '| t | [[Machine Learning\\|ML]] |\n'
                '`[[Machine Learning]]` stays\n```dataview\nLIST WHERE contains(file.outlinks, [[Machine Learning]])\n```')
        out, n = vault.fix_links_in(text, fix)
        self.assertEqual(n, 3)
        self.assertIn('See [[Machine_Learning|Machine Learning]] and [[Reinforcement_Learning|RL]].', out)
        self.assertIn('| t | [[Machine_Learning\\|ML]] |', out)
        self.assertIn('`[[Machine Learning]]` stays', out)
        self.assertIn('contains(file.outlinks, [[Machine Learning]])', out)


class Freshness(unittest.TestCase):
    def test_trim_related_removes_only_listed_targets(self):
        fm = 'title: T\nrelated:\n  - "[[Keep]]"\n  - "[[Stray Topic]]"\n  - "[[Stray Topic|alias]]"\ntags:\n  - "[[Stray Topic]]"'
        out, n = vault.trim_related_text(fm, {'stray topic'})
        self.assertEqual(n, 2)
        self.assertEqual(out, 'title: T\nrelated:\n  - "[[Keep]]"\ntags:\n  - "[[Stray Topic]]"')

    def test_as_of_needs_three_sources_in_the_right_format(self):
        v = vault.Vault(); schema = vault.load_yaml('schema.yml'); tm = vault.TagMap()
        fm = ('title: T\ndescription: d\ntags:\n  - ros\ntype: concept\nlevel: beginner\nsubfield:\n  - software\n'
              'status: draft\nauthor: Jordan_Smith\ndate: 2026-01-01\nas_of: 2026-13\nsources:\n  - "No year or url"\n'
              '  - "DeepMind (2025). MuJoCo. https://mujoco.org"\n')
        n = note('body', fm=fm); L = vault.Lint()
        vault.check_note(n, v, schema, tm, set(vault.load_yaml('subfields.yml')['subfields']), L)
        rules = {r for _, r, _, _ in L.items}
        self.assertTrue({'field-month', 'source-format', 'sources-required'} <= rules, rules)

    def test_generated_whats_new_page(self):
        out = vault.page_whats_new(vault.Book(vault.Vault()))
        self.assertIn('## Recently added', out)


class Tags(unittest.TestCase):
    def test_inline_tags_need_whitespace(self):
        n = note('#start and mid #ok, "#quoted" x#no `#code` #123 #v2/sub')
        self.assertEqual(n.inline_tags, ['start', 'ok', 'v2/sub'])

    def test_invalid_frontmatter_tags(self):
        n = note('x', fm='title: T\ntags:\n  - c++\n  - human factors\n  - ros\n')
        self.assertEqual(n.invalid_tags, ['c++', 'human factors'])
        self.assertEqual(n.tags, ['ros'])

    def test_tag_mapping(self):
        tm = vault.TagMap()
        self.assertEqual(tm.canonical('ROS2'), 'ros2')
        self.assertEqual(tm.canonical('Actuator'), 'actuators')
        self.assertEqual(tm.canonical('human factors'), 'human-factors')
        self.assertIsNone(tm.canonical('robotics'))
        self.assertIsNone(tm.canonical('tutorial'))   # becomes `type: tutorial`


if __name__ == '__main__':
    unittest.main()
